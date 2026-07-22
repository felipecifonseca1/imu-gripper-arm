#include "AttitudeEstimator.h"
#include "ESKFFilter.h"
#include <math.h>

using namespace Eigen;

/**
 * @brief Constructs ESKFFilter object and calls reset.
 */
ESKFFilter::ESKFFilter() {
    reset();
}

/**
 * @brief Resets filter nominal states and initial covariance matrix.
 */
void ESKFFilter::reset() {
    _q_nom = Quaternionf::Identity();
    _b_g = Vector3f::Zero();
    _a_lin = Vector3f::Zero();
    
    _P_cov.setZero();
    _P_cov.diagonal() << 0.000006092348396f, 0.000006092348396f, 0.000006092348396f,
                         0.000076154354947f, 0.000076154354947f, 0.000076154354947f,
                         0.00962361f       , 0.00962361f       , 0.00962361f;
                     
    _is_first_sample = true;
}

/**
 * @brief Gets current nominal quaternion components.
 * @param w Output scalar w component reference.
 * @param x Output vector x component reference.
 * @param y Output vector y component reference.
 * @param z Output vector z component reference.
 */
void ESKFFilter::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = _q_nom.w(); x = _q_nom.x(); y = _q_nom.y(); z = _q_nom.z();
}

/**
 * @brief Sets nominal quaternion state and normalizes.
 * @param w Quaternion scalar w component.
 * @param x Quaternion vector x component.
 * @param y Quaternion vector y component.
 * @param z Quaternion vector z component.
 */
void ESKFFilter::setQuaternion(float w, float x, float y, float z) {
    _q_nom.w() = w; _q_nom.x() = x; _q_nom.y() = y; _q_nom.z() = z;
    _q_nom.normalize();
}

/**
 * @brief Time Update (Prediction Phase)
 * Propagates the nominal state and expands uncertainty based on elapsed time.
 */
void ESKFFilter::predict(const Vector3f& gyro, float dt) {
    const float GyroscopeNoise = 2.91057927e-06f; 
    const float GyroscopeDriftNoise = 1e-7f;
    const float LinearAccelerationNoise = 1.0e-5f; 
    const float LinearAccelerationDecayFactor = 0.5f;

    // 1. Predict nominal orientation via Gyro kinematic integration
    Vector3f delta_theta = (gyro - _b_g) * dt;
    Quaternionf delta_q = fromRotationVector(delta_theta);
    _q_nom = _q_nom * delta_q;
    _q_nom.normalize();

    // 2. Predict linear acceleration decay profile
    _a_lin = LinearAccelerationDecayFactor * _a_lin;

    // 3. Analytical Error Covariance Propagation
    Matrix<float, 9, 9> P_old = _P_cov;
    _P_cov.setZero();
    
    // Propagate orientation error diagonal block
    _P_cov.block<3, 3>(0, 0) = P_old.block<3, 3>(0, 0) + 
                           (dt * dt) * (P_old.block<3, 3>(3, 3) + Matrix3f::Identity() * (GyroscopeDriftNoise + GyroscopeNoise));

    // Propagate gyro bias error diagonal block
    _P_cov.block<3, 3>(3, 3) = P_old.block<3, 3>(3, 3) + Matrix3f::Identity() * GyroscopeDriftNoise;

    // Cross-coupling off-diagonals
    Matrix3f off_diag = -dt * _P_cov.block<3, 3>(3, 3);
    _P_cov.block<3, 3>(3, 0) = off_diag;
    _P_cov.block<3, 3>(0, 3) = off_diag;

    // Propagate linear acceleration error block
    _P_cov.block<3, 3>(6, 6) = (LinearAccelerationDecayFactor * LinearAccelerationDecayFactor) * P_old.block<3, 3>(6, 6) + 
                           Matrix3f::Identity() * LinearAccelerationNoise;
}

/**
 * @brief Accelerometer Measurement Correction Step
 */
void ESKFFilter::updateAccelerometer(const Vector3f& a_m, float dt) {
    const float AccelerometerNoise = 3.23930639e-04f;
    const float GyroscopeNoise = 2.91057927e-06f; 
    const float GyroscopeDriftNoise = 1e-7f;
    const float LinearAccelerationNoise = 1.0e-5f;
    const float MaxOrientationCorrection = 0.03f;
    const float MaxGyroOffsetCorrection = 0.0002f;

    Matrix3f R_q = _q_nom.toRotationMatrix();
    Vector3f z_a_hat = rotMatToGravity(R_q);

    // Residual (Innovation)
    Vector3f r_a = (a_m + _a_lin) - z_a_hat;

    // Build specialized 3x9 Accelerometer Jacobian Matrix
    Matrix<float, 3, 9> H_a = Matrix<float, 3, 9>::Zero();
    H_a.block<3, 3>(0, 0) = -buildHPart(z_a_hat);
    H_a.block<3, 3>(0, 3) = dt * buildHPart(z_a_hat);
    H_a.block<3, 3>(0, 6) = -Matrix3f::Identity();

    // Measurement uncertainty setup
    float accelMeasNoiseVar = AccelerometerNoise + LinearAccelerationNoise + (dt * dt) * (GyroscopeDriftNoise + GyroscopeNoise);
    Matrix3f R_a = Matrix3f::Identity() * accelMeasNoiseVar;

    // Compute Kalman Gain via a simple 3x3 matrix inversion
    Matrix3f S_a = H_a * _P_cov * H_a.transpose() + R_a;
    Matrix<float, 9, 3> K_a = _P_cov * H_a.transpose() * S_a.inverse();
    Matrix<float, 9, 1> delta_x = K_a * r_a;

    // Apply error vector injection to nominal states
    Vector3f delta_theta_hat = limitVectorNorm(delta_x.segment<3>(0), MaxOrientationCorrection);
    Vector3f delta_b_g_hat = limitVectorNorm(delta_x.segment<3>(3), MaxGyroOffsetCorrection);
    Vector3f delta_a_lin_hat = delta_x.segment<3>(6);

    _q_nom = _q_nom * fromRotationVector(delta_theta_hat);
    _q_nom.normalize();

    _b_g += delta_b_g_hat;
    _a_lin += delta_a_lin_hat;

    // Update 9x9 Error Covariance
    _P_cov = (Matrix<float, 9, 9>::Identity() - K_a * H_a) * _P_cov;
}

/**
 * @brief Magnetometer Measurement Correction Step
 */
void ESKFFilter::updateMagnetometer(const Vector3f& m_m, float dt) {
    const float MagnetometerNoise = 0.01f;
    const float MaxOrientationCorrection = 0.03f;
    const float MaxGyroOffsetCorrection = 0.0002f;

    Matrix3f R_q = _q_nom.toRotationMatrix();
    Vector3f m_m_norm = m_m.normalized();
    
    // Dynamic decoupled heading estimation reference projection
    Vector3f m_world = R_q * m_m_norm;
    Vector3f m_ref_world(sqrt(m_world(0)*m_world(0) + m_world(1)*m_world(1)), 0.0f, m_world(2));
    Vector3f m_s_hat = R_q.transpose() * m_ref_world;

    // Residual (Innovation)
    Vector3f r_m = m_m_norm - m_s_hat;

    // Build specialized 3x9 Magnetometer Jacobian Matrix
    Matrix<float, 3, 9> H_m = Matrix<float, 3, 9>::Zero();
    H_m.block<3, 3>(0, 0) = -buildHPart(m_s_hat);
    H_m.block<3, 3>(0, 3) = dt * buildHPart(m_s_hat);

    Matrix3f R_m = Matrix3f::Identity() * MagnetometerNoise;

    // Compute Kalman Gain via a simple 3x3 matrix inversion
    Matrix3f S_m = H_m * _P_cov * H_m.transpose() + R_m;
    Matrix<float, 9, 3> K_m = _P_cov * H_m.transpose() * S_m.inverse();
    Matrix<float, 9, 1> delta_x = K_m * r_m;

    // Apply error vector injection to nominal states
    Vector3f delta_theta_hat = limitVectorNorm(delta_x.segment<3>(0), MaxOrientationCorrection);
    Vector3f delta_b_g_hat = limitVectorNorm(delta_x.segment<3>(3), MaxGyroOffsetCorrection);

    _q_nom = _q_nom * fromRotationVector(delta_theta_hat);
    _q_nom.normalize();

    _b_g += delta_b_g_hat;

    // Update 9x9 Error Covariance
    _P_cov = (Matrix<float, 9, 9>::Identity() - K_m * H_m) * _P_cov;
}

/**
 * @brief Backward-Compatible Interface Update Method
 * Routes data cascadingly through the independent pipelines.
 */
void ESKFFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {
    Vector3f a_m(ax, ay, az);
    Vector3f omega_m(gx, gy, gz);
    Vector3f m_m(mx, my, mz);

    if (_is_first_sample) {
        _q_nom = ecompass(a_m, m_m);
        _is_first_sample = false;
    }

    // 1. High-frequency prediction
    predict(omega_m, dt);

    // 2. Sequential Accel Update
    if (!ignoreAccel) {
        updateAccelerometer(a_m, dt);
    }

    // 3. Sequential Magnetometer Update
    if (m_m.squaredNorm() > 1e-6f) {
        updateMagnetometer(m_m, dt);
    }

    // 4. Complementary blending pass (only if both updates were valid/present)
    if (!ignoreAccel && m_m.squaredNorm() > 1e-6f) {
        const float OrientationCorrectionGain = 0.1f;
        const float MaxOrientationCorrection = 0.03f;

        Quaternionf q_meas = ecompass(a_m, m_m);
        Quaternionf q_delta = _q_nom.conjugate() * q_meas;
        
        AngleAxisf aa_delta(q_delta);
        Vector3f delta_theta_meas = aa_delta.axis() * aa_delta.angle();
        for (int i = 0; i < 3; ++i) {
            if (delta_theta_meas(i) > M_PI) delta_theta_meas(i) -= 2.0f * M_PI;
            if (delta_theta_meas(i) < -M_PI) delta_theta_meas(i) += 2.0f * M_PI;
        }
        delta_theta_meas = limitVectorNorm(delta_theta_meas, MaxOrientationCorrection);
        
        Quaternionf delta_q_meas = fromRotationVector(OrientationCorrectionGain * delta_theta_meas);
        _q_nom = _q_nom * delta_q_meas;
        _q_nom.normalize();
    }
}

/**
 * @brief Builds skew-symmetric matrix part for measurement Jacobian.
 * @param v Input 3D vector.
 * @return 3x3 skew-symmetric matrix.
 */
Eigen::Matrix3f ESKFFilter::buildHPart(const Eigen::Vector3f& v) {
    Eigen::Matrix3f h = Eigen::Matrix3f::Zero();
    h(0, 1) = v(2); h(0, 2) = -v(1); h(1, 2) = v(0);
    return h - h.transpose();
}

/**
 * @brief Projects rotation matrix to expected unit gravity direction vector.
 * @param R Body-to-world rotation matrix.
 * @return Projected gravity vector.
 */
Eigen::Vector3f ESKFFilter::rotMatToGravity(const Eigen::Matrix3f& R) {
    const float gravity = 1.0f;
    return gravity * R.row(2).transpose();
}

/**
 * @brief Projects rotation matrix to expected unit magnetic field vector.
 * @param R Body-to-world rotation matrix.
 * @return Projected magnetic unit vector.
 */
Eigen::Vector3f ESKFFilter::rotMatToMagnetic(const Eigen::Matrix3f& R) {
    Eigen::Vector3f m = R.row(0).transpose();
    float norm = m.norm();
    if (norm == 0.0f) return m;
    return m / norm;
}

/**
 * @brief Calculates initial orientation quaternion from accel and mag vectors.
 * @param a Accelerometer measurement vector.
 * @param m Magnetometer measurement vector.
 * @return Initial attitude unit quaternion.
 */
Eigen::Quaternionf ESKFFilter::ecompass(const Eigen::Vector3f& a, const Eigen::Vector3f& m) {
    Eigen::Vector3f a_norm = a.normalized();
    Eigen::Vector3f cross_am = a_norm.cross(m);
    if (cross_am.norm() < 1e-6f) return Eigen::Quaternionf::Identity();
    Eigen::Vector3f R2 = cross_am.normalized();
    Eigen::Vector3f R1 = R2.cross(a_norm).normalized();
    Eigen::Matrix3f R;
    R.row(0) = R1; R.row(1) = R2; R.row(2) = a_norm;
    return Eigen::Quaternionf(R);
}

/**
 * @brief Converts rotation vector to delta quaternion representation.
 * @param r 3D rotation vector.
 * @return Equivalent delta quaternion.
 */
Eigen::Quaternionf ESKFFilter::fromRotationVector(const Eigen::Vector3f& r) {
    float theta = r.norm();
    if (theta < 1e-6f) return Eigen::Quaternionf::Identity();
    float sin_half = sin(theta / 2.0f);
    float cos_half = cos(theta / 2.0f);
    Eigen::Vector3f axis = r / theta;
    return Eigen::Quaternionf(cos_half, axis(0) * sin_half, axis(1) * sin_half, axis(2) * sin_half);
}

/**
 * @brief Clamps vector magnitude to max_norm threshold.
 * @param v Input 3D vector.
 * @param max_norm Maximum magnitude threshold.
 * @return Magnitude-limited 3D vector.
 */
Eigen::Vector3f ESKFFilter::limitVectorNorm(const Eigen::Vector3f& v, float max_norm) {
    float norm = v.norm();
    if (norm > max_norm) return v * (max_norm / norm);
    return v;
}