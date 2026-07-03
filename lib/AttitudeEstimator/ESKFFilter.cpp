#include "AttitudeEstimator.h"
#include "ESKFFilter.h"
#include <math.h>

using namespace Eigen;

ESKFFilter::ESKFFilter() {
    reset();
}

void ESKFFilter::reset() {
    _q_nom = Quaternionf::Identity();
    _b_g = Vector3f::Zero();
    _a_lin = Vector3f::Zero();
    
    // Empirical initial covariance matrix
    _P_cov.setZero();
    _P_cov.diagonal() << 0.000006092348396f, 0.000006092348396f, 0.000006092348396f,
                         0.000076154354947f, 0.000076154354947f, 0.000076154354947f,
                         0.00962361f       , 0.00962361f       , 0.00962361f;
                     
    _is_first_sample = true;
}

void ESKFFilter::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = _q_nom.w();
    x = _q_nom.x();
    y = _q_nom.y();
    z = _q_nom.z();
}

void ESKFFilter::setQuaternion(float w, float x, float y, float z) {
    _q_nom.w() = w;
    _q_nom.x() = x;
    _q_nom.y() = y;
    _q_nom.z() = z;
    _q_nom.normalize();
}

void ESKFFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {

    const float AccelerometerNoise = 2.61980858e-06f; // Measured
    const float GyroscopeNoise = 7.86768851e-07f; // Measured
    const float GyroscopeDriftNoise = 1e-7f;
    const float LinearAccelerationNoise = 1.0e-5f; // Slightly higher than AccelNoise
    const float MagnetometerNoise = 1.35611117e+00f; // Measured
    const float LinearAccelerationDecayFactor = 0.5f;
    const float OrientationCorrectionGain = 0.1f;
    const float MaxOrientationCorrection = 0.03f;
    const float MaxGyroOffsetCorrection = 0.0002f;

    Vector3f a_m(ax, ay, az);
    Vector3f omega_m(gx, gy, gz);
    Vector3f m_m(mx, my, mz);

    // Initial orientation via ecompass
    if (_is_first_sample) {
        _q_nom = ecompass(a_m, m_m);
        _is_first_sample = false;
    }

    // 1. Predict nominal orientation
    Vector3f delta_theta = (omega_m - _b_g) * dt;
    Quaternionf delta_q = fromRotationVector(delta_theta);
    Quaternionf q_hat_minus = _q_nom * delta_q;
    q_hat_minus.normalize();

    // 2. Predict linear acceleration
    Vector3f a_lin_hat_minus = LinearAccelerationDecayFactor * _a_lin;

    // Convert orientation to rotation matrix
    Matrix3f R_q_hat_minus = q_hat_minus.toRotationMatrix();

    // Predict measurement directions
    Vector3f z_a_hat_minus = rotMatToGravity(R_q_hat_minus);
    Vector3f m_s_hat_minus = rotMatToMagnetic(R_q_hat_minus);

    // Residuals
    // Python gravity sign for NED reference frame is 1 (Positive gravity down)
    Vector3f z_a = a_m + a_lin_hat_minus;
    Vector3f r_a = z_a - z_a_hat_minus;

    Vector3f m_m_norm = m_m.normalized();
    Vector3f r_m = m_m_norm - m_s_hat_minus;

    // 3. Assemble Jacobian H_k (6x9)
    Matrix<float, 6, 9> H_k = Matrix<float, 6, 9>::Zero();
    
    Matrix3f H_a_theta = -buildHPart(z_a_hat_minus);
    Matrix3f H_a_bias = dt * buildHPart(z_a_hat_minus);
    H_k.block<3, 3>(0, 0) = H_a_theta;
    H_k.block<3, 3>(0, 3) = H_a_bias;
    
    Matrix3f H_m_theta = -buildHPart(m_s_hat_minus);
    Matrix3f H_m_bias = dt * buildHPart(m_s_hat_minus);
    H_k.block<3, 3>(3, 0) = H_m_theta;
    H_k.block<3, 3>(3, 3) = H_m_bias;

    // 4. Kalman Gain & Innovation
    Matrix<float, 6, 1> r_k;
    r_k.segment<3>(0) = r_a;
    r_k.segment<3>(3) = r_m;

    float accelMeasNoiseVar = AccelerometerNoise + LinearAccelerationNoise + (dt * dt) * (GyroscopeDriftNoise + GyroscopeNoise);
    
    Matrix<float, 6, 6> R_k = Matrix<float, 6, 6>::Zero();
    R_k.block<3, 3>(0, 0) = Matrix3f::Identity() * accelMeasNoiseVar;
    R_k.block<3, 3>(3, 3) = Matrix3f::Identity() * MagnetometerNoise;

    Matrix<float, 6, 6> S_k = H_k * _P_cov * H_k.transpose() + R_k;
    Matrix<float, 9, 6> K_k = _P_cov * H_k.transpose() * S_k.inverse();
    Matrix<float, 9, 1> delta_x_hat = K_k * r_k;

    // 5. Correct error estimates & nominal state
    Vector3f delta_theta_hat = limitVectorNorm(delta_x_hat.segment<3>(0), MaxOrientationCorrection);
    Vector3f delta_b_g_hat = limitVectorNorm(delta_x_hat.segment<3>(3), MaxGyroOffsetCorrection);
    Vector3f delta_a_lin_hat = delta_x_hat.segment<3>(6);

    // Apply orientation correction
    Quaternionf delta_q_corr = fromRotationVector(-delta_theta_hat);
    _q_nom = q_hat_minus * delta_q_corr;
    _q_nom.normalize();

    // Complementary ecompass step
    Quaternionf q_meas = ecompass(a_m, m_m);
    Quaternionf q_delta = _q_nom.conjugate() * q_meas;
    
    // Extract rotation vector from q_delta
    AngleAxisf aa_delta(q_delta);
    Vector3f delta_theta_meas = aa_delta.axis() * aa_delta.angle();
    // Wrap to [-pi, pi]
    for (int i = 0; i < 3; ++i) {
        if (delta_theta_meas(i) > M_PI) delta_theta_meas(i) -= 2.0f * M_PI;
        if (delta_theta_meas(i) < -M_PI) delta_theta_meas(i) += 2.0f * M_PI;
    }
    delta_theta_meas = limitVectorNorm(delta_theta_meas, MaxOrientationCorrection);
    
    Quaternionf delta_q_meas = fromRotationVector(OrientationCorrectionGain * delta_theta_meas);
    _q_nom = _q_nom * delta_q_meas;
    _q_nom.normalize();

    // Correct remaining nominal states
    _b_g = _b_g - delta_b_g_hat;
    _a_lin = a_lin_hat_minus - delta_a_lin_hat;

    // 6. Covariance propagation
    Matrix<float, 9, 9> P_plus = (Matrix<float, 9, 9>::Identity() - K_k * H_k) * _P_cov;
    
    _P_cov.setZero();
    // Block 1 (Orientation error covariance propagation)
    _P_cov.block<3, 3>(0, 0) = P_plus.block<3, 3>(0, 0) + 
                           (dt * dt) * (P_plus.block<3, 3>(3, 3) + Matrix3f::Identity() * (GyroscopeDriftNoise + GyroscopeNoise));

    // Block 2 (Gyro bias error covariance propagation)
    _P_cov.block<3, 3>(3, 3) = P_plus.block<3, 3>(3, 3) + Matrix3f::Identity() * GyroscopeDriftNoise;

    // Off-diagonals between orientation and bias
    Matrix3f off_diag = -dt * _P_cov.block<3, 3>(3, 3);
    _P_cov.block<3, 3>(3, 0) = off_diag;
    _P_cov.block<3, 3>(0, 3) = off_diag;

    // Block 3 (Linear acceleration covariance propagation)
    _P_cov.block<3, 3>(6, 6) = (LinearAccelerationDecayFactor * LinearAccelerationDecayFactor) * P_plus.block<3, 3>(6, 6) + 
                           Matrix3f::Identity() * LinearAccelerationNoise;
}

Eigen::Matrix3f ESKFFilter::buildHPart(const Eigen::Vector3f& v) {
    Eigen::Matrix3f h = Eigen::Matrix3f::Zero();
    h(0, 1) = v(2);
    h(0, 2) = -v(1);
    h(1, 2) = v(0);
    return h - h.transpose();
}

Eigen::Vector3f ESKFFilter::rotMatToGravity(const Eigen::Matrix3f& R) {
    // In NED reference frame, gravity points down (along Z axis)
    const float gravity = 9.80665f;
    return gravity * R.col(2);
}

Eigen::Vector3f ESKFFilter::rotMatToMagnetic(const Eigen::Matrix3f& R) {
    // Magnetic reference vector points North (along X axis)
    Eigen::Vector3f m = R.col(0);
    float norm = m.norm();
    if (norm == 0.0f) return m;
    return m / norm;
}

Eigen::Quaternionf ESKFFilter::ecompass(const Eigen::Vector3f& a, const Eigen::Vector3f& m) {
    Eigen::Vector3f a_norm = a.normalized();
    Eigen::Vector3f cross_am = a_norm.cross(m);
    if (cross_am.norm() < 1e-6f) {
        return Eigen::Quaternionf::Identity();
    }
    Eigen::Vector3f R2 = cross_am.normalized();
    Eigen::Vector3f R1 = R2.cross(a_norm).normalized();
    
    Eigen::Matrix3f R;
    R.col(0) = R1;
    R.col(1) = R2;
    R.col(2) = a_norm;
    
    return Eigen::Quaternionf(R);
}

Eigen::Quaternionf ESKFFilter::fromRotationVector(const Eigen::Vector3f& r) {
    float theta = r.norm();
    if (theta < 1e-6f) {
        return Eigen::Quaternionf::Identity();
    }
    float sin_half = sin(theta / 2.0f);
    float cos_half = cos(theta / 2.0f);
    Eigen::Vector3f axis = r / theta;
    return Eigen::Quaternionf(cos_half, axis(0) * sin_half, axis(1) * sin_half, axis(2) * sin_half);
}

Eigen::Vector3f ESKFFilter::limitVectorNorm(const Eigen::Vector3f& v, float max_norm) {
    float norm = v.norm();
    if (norm > max_norm) {
        return v * (max_norm / norm);
    }
    return v;
}
