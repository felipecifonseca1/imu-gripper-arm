// #include "AttitudeEstimator.h"
// #include <math.h>

// using namespace Eigen;

// /**
//  * @brief Default constructor.
//  * @details Initializes the nominal quaternion to identity and bias to zero.
//  */
// MEKF::MEKF() {
//     _useMagnetometer = true;
//     _mekf_q_proc = 0.001f;
//     _mekf_r_accel = 0.1f;
//     _mekf_r_mag = 5.0f;
//     _mag_ref_x = 1.0f;
//     _mag_ref_y = 0.0f;
//     _mag_ref_z = 0.0f;

//     resetState(); // Default to identity (upright)
    
//     // Default Process Noise 
//     _proc = Matrix<float, 6, 6>::Identity() * 0.001f;
    
//     // Default Error Covariance (Initial uncertainty)
//     _cov = Matrix<float, 6, 6>::Identity() * 0.1f;
// }

// /**
//  * @brief Resets the internal state to initial values.
//  * @param physicalZAxisDown If true, initializes with a 180-degree flip around the X-axis.
//  */
// void MEKF::resetState() {
//     // Standard identity: q = [1, 0, 0, 0]
//     _quat << 1.0f, 0.0f, 0.0f, 0.0f;
//     _bias.setZero();
// }

// /**
//  * @brief Initializes the filter with process and initial error covariances.
//  * @param Q_in  Process noise covariance (6x6).
//  * @param P0_in Initial error covariance (6x6).
//  */
// void MEKF::init(const Matrix<float, 6, 6>& Q_in, const Matrix<float, 6, 6>& P0_in) {
//     _proc = Q_in;
//     _cov  = P0_in;
//     resetState();
// }

// void MEKF::setProcessNoise(float q_diagonal) {
//     _proc = Matrix<float, 6, 6>::Identity() * q_diagonal;
// }


// /**
//  * @brief Predict Step: Propagates the quaternion and error covariance using gyroscope data.
//  * @param gyro Gyroscope reading in rad/s [x, y, z].
//  * @param dt Delta time in seconds.
//  */
// void MEKF::predict(const Matrix<float, 3, 1>& gyro, float dt) {
//     // 1. Unbias the gyro reading using current bias estimate
//     Matrix<float, 3, 1> omega = gyro - _bias;

//     // 2. Propagate the nominal quaternion
//     Matrix<float, 3, 1> delta_theta = omega * dt;
//     float angle = delta_theta.norm();

//     Matrix<float, 4, 1> dq;
//     if (angle > 1e-5f) {
//         float half_angle = angle * 0.5f;
//         float s = sinf(half_angle) / angle;
//         dq(0) = cosf(half_angle);
//         dq(1) = delta_theta(0) * s;
//         dq(2) = delta_theta(1) * s;
//         dq(3) = delta_theta(2) * s;
//     } else {
//         dq(0) = 1.0f;
//         dq(1) = delta_theta(0) * 0.5f;
//         dq(2) = delta_theta(1) * 0.5f;
//         dq(3) = delta_theta(2) * 0.5f;
//     }

//     _quat = quatMultiply(_quat, dq);
//     _quat.normalize();

//     // 3. Construct the State Transition Jacobian (F)
//     Matrix<float, 6, 6> F = Matrix<float, 6, 6>::Identity();
//     F.block<3, 3>(0, 0) -= skewSymmetric(omega) * dt; 
//     F.block<3, 3>(0, 3)  = -Matrix<float, 3, 3>::Identity() * dt; 

//     // 4. Propagate Error Covariance: P = F * P * F^T + Q
//     _cov = F * _cov * F.transpose() + _proc;
// }

// /**
//  * @brief Update step: Corrects the state using a directional sensor (Accel or Mag).
//  * @param measurement The raw sensor vector (unnormalized).
//  * @param reference The reference vector in world frame (e.g., [0,0,1] for gravity).
//  * @param R_meas Measurement noise covariance matrix for this sensor (3x3).
//  */
// void MEKF::updateMeasurement(const Matrix<float, 3, 1>& measurement,
//                               const Matrix<float, 3, 1>& reference,
//                               const Matrix<float, 3, 3>& R_meas) {

//     if (measurement.squaredNorm() < 1e-6f) return;

//     // 1. Calculate Expected Measurement in the Body Frame
//     Matrix<float, 3, 3> C = quatToRotationMatrix(_quat); 
//     Matrix<float, 3, 1> h = C.transpose() * reference.normalized();

//     // 2. Construct Measurement Jacobian (H)
//     Matrix<float, 3, 6> H = Matrix<float, 3, 6>::Zero();
//     H.block<3, 3>(0, 0) = skewSymmetric(h);

//     // 3. Kalman Gain Calculation
//     Matrix<float, 3, 3> S = H * _cov * H.transpose() + R_meas;
//     Matrix<float, 6, 3> K = _cov * H.transpose() * S.inverse();

//     // 4. Calculate Innovation (Residual)
//     Matrix<float, 3, 1> innovation = measurement.normalized() - h;

//     // 5. Calculate Error State Correction
//     Matrix<float, 6, 1> dx = K * innovation;
//     Matrix<float, 3, 1> attitude_error = dx.segment<3>(0);
//     Matrix<float, 3, 1> bias_error     = dx.segment<3>(3);

//     // 6. *** MULTIPLICATIVE UPDATE ***
//     Matrix<float, 4, 1> dq_update;
//     dq_update(0) = 1.0f;
//     dq_update(1) = attitude_error(0) * 0.5f;
//     dq_update(2) = attitude_error(1) * 0.5f;
//     dq_update(3) = attitude_error(2) * 0.5f;

//     _quat = quatMultiply(_quat, dq_update);
//     _quat.normalize();

//     // Additive update for the gyro bias
//     _bias += bias_error;

//     // 7. Update Covariance (Joseph Form for numerical stability)
//     Matrix<float, 6, 6> I6   = Matrix<float, 6, 6>::Identity();
//     Matrix<float, 6, 6> I_KH = I6 - K * H;
//     _cov = I_KH * _cov * I_KH.transpose() + K * R_meas * K.transpose();
// }

// /**
//  * @brief Specialized update that ONLY affects Heading (Yaw), decoupled from Tilt.
//  * @param mag_measure The raw magnetometer reading in body frame.
//  * @param mag_ref     The magnetic reference vector in world frame.
//  * @param r_yaw       Noise variance for the heading measurement.
//  */
// void MEKF::updateMagnetometerYaw(const Matrix<float, 3, 1>& mag_measure,
//                                const Matrix<float, 3, 1>& mag_ref,
//                                float r_yaw) {
//     if (mag_measure.squaredNorm() < 1e-6f) return;

//     // 1. Current Body-to-World Orientation
//     Matrix<float, 3, 3> C = quatToRotationMatrix(_quat);

//     // 2. Project Body-Mag into World Frame to find Measured Heading
//     Matrix<float, 3, 1> m_world = C * mag_measure.normalized();
//     float psi_meas = atan2f(m_world(1), m_world(0));
//     float psi_ref  = atan2f(mag_ref(1), mag_ref(0));

//     // 3. Innovation (Yaw Error)
//     float innovation = psi_meas - psi_ref;
//     while (innovation >  3.14159265f) innovation -= 6.28318531f; 
//     while (innovation < -3.14159265f) innovation += 6.28318531f; 

//     // 4. Construct Jacobian (H)
//     // We observe the rotation error around the World-Z axis.
//     // Mapping world-Z error into body error coordinates: h_body = C.row(2).
//     Matrix<float, 1, 6> H = Matrix<float, 1, 6>::Zero();
//     H(0, 0) = C(2, 0); 
//     H(0, 1) = C(2, 1);
//     H(0, 2) = C(2, 2);

//     // 5. Kalman Gain Calculation (1D Update)
//     float S = (H * _cov * H.transpose())(0,0) + r_yaw;
//     Matrix<float, 6, 1> K = _cov * H.transpose() / S;

//     // 6. Update State
//     Matrix<float, 6, 1> dx = K * innovation;
//     Matrix<float, 3, 1> attitude_error = dx.segment<3>(0);
//     Matrix<float, 3, 1> bias_error     = dx.segment<3>(3);

//     Matrix<float, 4, 1> dq_update;
//     dq_update(0) = 1.0f;
//     dq_update(1) = attitude_error(0) * 0.5f;
//     dq_update(2) = attitude_error(1) * 0.5f;
//     dq_update(3) = attitude_error(2) * 0.5f;

//     _quat = quatMultiply(_quat, dq_update);
//     _quat.normalize();
//     _bias += bias_error;

//     // 7. Update Covariance
//     Matrix<float, 6, 6> I6 = Matrix<float, 6, 6>::Identity();
//     _cov = (I6 - K * H) * _cov;
// }
// Matrix<float, 3, 3> MEKF::skewSymmetric(const Matrix<float, 3, 1>& v) const {
//     Matrix<float, 3, 3> S;
//     S <<   0.0f, -v(2),  v(1),
//           v(2),   0.0f, -v(0),
//          -v(1),   v(0),  0.0f;
//     return S;
// }

// /**
//  * @brief Hamilton product of two quaternions: q1 * q2.
//  */
// Matrix<float, 4, 1> MEKF::quatMultiply(const Matrix<float, 4, 1>& q1,
//                                                 const Matrix<float, 4, 1>& q2) const {
//     Matrix<float, 4, 1> r;
//     r(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3); // w
//     r(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2); // x
//     r(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1); // y
//     r(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0); // z
//     return r;
// }

// /**
//  * @brief Converts a quaternion to its equivalent rotation matrix R_world_from_body (body -> world).
//  */
// Matrix<float, 3, 3> MEKF::quatToRotationMatrix(const Matrix<float, 4, 1>& qv) const {
//     float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
//     Matrix<float, 3, 3> R;
//     R << 1.0f - 2.0f*(qy*qy + qz*qz),  2.0f*(qx*qy - qz*qw),        2.0f*(qx*qz + qy*qw),
//          2.0f*(qx*qy + qz*qw),          1.0f - 2.0f*(qx*qx + qz*qz), 2.0f*(qy*qz - qx*qw),
//          2.0f*(qx*qz - qy*qw),          2.0f*(qy*qz + qx*qw),        1.0f - 2.0f*(qx*qx + qy*qy);
//     return R;
// }

// // --- IAttitudeFilter Overrides & Wrapper functions ---

// void MEKF::reset() {
//     resetState();
// }

// void MEKF::getQuaternion(float& w, float& x, float& y, float& z) const {
//     w = _quat(0); x = _quat(1); y = _quat(2); z = _quat(3);
// }

// void MEKF::setQuaternion(float w, float x, float y, float z) {
//     _quat << w, x, y, z;
//     _quat.normalize();
// }

// void MEKF::setMEKFTuning(float q_proc, float r_accel, float r_mag) {
//     _mekf_q_proc = q_proc;
//     _mekf_r_accel = r_accel;
//     _mekf_r_mag = r_mag;
//     setProcessNoise(q_proc);
// }

// void MEKF::setUseMagnetometer(bool use) {
//     _useMagnetometer = use;
// }

// void MEKF::setMagneticReference(float mx, float my, float mz) {
//     _mag_ref_x = mx;
//     _mag_ref_y = my;
//     _mag_ref_z = mz;
// }

// void MEKF::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {
//     using Vec3 = Eigen::Matrix<float, 3, 1>;
//     using Mat3 = Eigen::Matrix<float, 3, 3>;

//     // --- Predict ---
//     Vec3 gyro(gx, gy, gz);
//     predict(gyro, dt);

//     // --- Accelerometer update ---
//     if (!ignoreAccel) {
//         Vec3 accel(ax, ay, az);
//         Vec3 ref_gravity(0.0f, 0.0f, 1.0f);
//         Mat3 R_accel = Mat3::Identity() * _mekf_r_accel; 
//         updateMeasurement(accel, ref_gravity, R_accel);
//     }

//     // --- Magnetometer update ---
//     if (_useMagnetometer) {
//         Vec3 mag(mx, my, mz);
//         Vec3 ref_mag(_mag_ref_x, _mag_ref_y, _mag_ref_z);
//         updateMagnetometerYaw(mag, ref_mag, _mekf_r_mag);
//     }
// }

#include "AttitudeEstimator.h"
#include "MEKF.h"

using namespace Eigen;

/**
 * @brief Constructs MEKF object with default initial tuning parameters.
 */
MEKF::MEKF() {
    _useMagnetometer = true;
    _mekf_q_proc = 0.001f;
    _mekf_r_accel = 0.1f;
    _mekf_r_mag = 5.0f;
    _mag_ref_x = 1.0f;
    _mag_ref_y = 0.0f;
    _mag_ref_z = 0.0f;

    _Q_theta = Matrix3f::Identity() * _mekf_q_proc;
    _Q_bias = Matrix3f::Identity() * (_mekf_q_proc * 0.1f);

    resetState();
    _cov = Matrix<float, 6, 6>::Identity() * 0.1f;
}

/**
 * @brief Resets quaternion to identity and bias to zero.
 */
void MEKF::resetState() {
    _quat << 1.0f, 0.0f, 0.0f, 0.0f;
    _bias << 0.0f, 0.0f, 0.0f;
}

/**
 * @brief Resets state and error covariance.
 */
void MEKF::reset() {
    resetState();
    _cov = Matrix<float, 6, 6>::Identity() * 0.1f;
}

/**
 * @brief Gets current estimated unit quaternion values.
 * @param w Output w component.
 * @param x Output x component.
 * @param y Output y component.
 * @param z Output z component.
 */
void MEKF::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = _quat(0); x = _quat(1); y = _quat(2); z = _quat(3);
}

/**
 * @brief Sets unit quaternion state.
 * @param w Scalar w.
 * @param x Vector x.
 * @param y Vector y.
 * @param z Vector z.
 */
void MEKF::setQuaternion(float w, float x, float y, float z) {
    _quat << w, x, y, z;
    _quat.normalize();
}

/**
 * @brief Sets MEKF tuning parameters.
 * @param q_proc Process noise parameter.
 * @param r_accel Accelerometer noise parameter.
 * @param r_mag Magnetometer noise parameter.
 */
void MEKF::setMEKFTuning(float q_proc, float r_accel, float r_mag) {
    _mekf_q_proc = q_proc;
    _mekf_r_accel = r_accel;
    _mekf_r_mag = r_mag;
    setProcessNoise(q_proc);
}

/**
 * @brief Configures process noise diagonal matrices.
 * @param q_diagonal Process noise magnitude factor.
 */
void MEKF::setProcessNoise(float q_diagonal) {
    _Q_theta = Matrix3f::Identity() * (q_diagonal * 0.1f); // Time-scaled in predict
    _Q_bias = Matrix3f::Identity() * (q_diagonal * 0.01f);
}

/**
 * @brief Sets magnetometer usage flag.
 * @param use True to use magnetometer.
 */
void MEKF::setUseMagnetometer(bool use) {
    _useMagnetometer = use;
}

/**
 * @brief Sets world frame magnetic reference vector.
 * @param mx Reference vector X.
 * @param my Reference vector Y.
 * @param mz Reference vector Z.
 */
void MEKF::setMagneticReference(float mx, float my, float mz) {
    _mag_ref_x = mx;
    _mag_ref_y = my;
    _mag_ref_z = mz;
}

/**
 * @brief Enforces mathematical symmetry on error covariance matrix.
 */
void MEKF::forceSymmetry() {
    // Only process the upper triangle to force the lower triangle to match
    _cov = 0.5f * (_cov + _cov.transpose().eval());
}

/**
 * @brief Calculates 3x3 skew-symmetric matrix for 3D vector.
 * @param v Input 3D vector.
 * @return 3x3 skew-symmetric matrix.
 */
Matrix3f MEKF::skewSymmetric(const Vector3f& v) {
    Matrix3f S;
    S <<  0.0f, -v(2),  v(1),
          v(2),  0.0f, -v(0),
         -v(1),  v(0),  0.0f;
    return S;
}

/**
 * @brief Computes Hamilton quaternion multiplication.
 * @param q1 First quaternion.
 * @param q2 Second quaternion.
 * @return Product quaternion vector.
 */
Vector4f MEKF::quatMult(const Vector4f& q1, const Vector4f& q2) {
    Vector4f q_out;
    q_out(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
    q_out(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
    q_out(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
    q_out(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
    return q_out;
}

// ---------------------------------------------------------
// OPTIMIZED ANALYTICAL PREDICT METHOD 
// ---------------------------------------------------------
/**
 * @brief High-frequency prediction phase propagating nominal orientation and covariance.
 * @param gyro Gyroscope measurement vector in rad/s.
 * @param dt Delta time step in seconds.
 */
void MEKF::predict(const Vector3f& gyro, float dt) {
    // 1. Bias-compensated gyro vector
    Vector3f omega = gyro - _bias;

    // 2. Propagate Nominal State (Quaternion Kinematics)
    float omega_norm = omega.norm();
    Vector4f dq;
    if (omega_norm > 1e-5f) {
        float half_theta = 0.5f * omega_norm * dt;
        float s = sin(half_theta) / omega_norm;
        dq << cos(half_theta), omega(0)*s, omega(1)*s, omega(2)*s;
    } else {
        dq << 1.0f, 0.0f, 0.0f, 0.0f;
    }
    _quat = quatMult(_quat, dq);
    _quat.normalize();

    // 3. Analytical Error Covariance Propagation
    // Extract the four 3x3 block quadrants from the 6x6 P matrix
    Matrix3f P00 = _cov.block<3, 3>(0, 0); // Orientation Uncertainty
    Matrix3f P01 = _cov.block<3, 3>(0, 3); // Cross-coupling
    Matrix3f P11 = _cov.block<3, 3>(3, 3); // Gyro Bias Uncertainty

    // Orientation error state transition block (Phi_11)
    Matrix3f Phi_11 = Matrix3f::Identity() - skewSymmetric(omega) * dt;

    // Analytically solve the matrix expansion: F * P * F^T + Q
    Matrix3f term1 = Phi_11 * P01 * dt;
    
    // Top-Left Block (New Orientation Uncertainty)
    Matrix3f new_P00 = Phi_11 * P00 * Phi_11.transpose() - term1 - term1.transpose() + (P11 * (dt * dt)) + (_Q_theta * dt);
    
    // Top-Right Block (New Cross-Coupling)
    Matrix3f new_P01 = (Phi_11 * P01) - (P11 * dt);

    // Bottom-Right Block (New Bias Uncertainty)
    Matrix3f new_P11 = P11 + (_Q_bias * dt);

    // Inject the calculated blocks directly back into the 6x6 memory buffer
    _cov.block<3, 3>(0, 0) = new_P00;
    _cov.block<3, 3>(0, 3) = new_P01;
    _cov.block<3, 3>(3, 0) = new_P01.transpose(); // P10 is structurally just P01^T
    _cov.block<3, 3>(3, 3) = new_P11;
    
    // HACK: Force symmetry to prevent matrix from becoming non-symmetric
    // This is crucial because Eigen's direct block assignment above creates a 
    // mathematical mismatch between the upper and lower triangles.
    forceSymmetry();
}

// ---------------------------------------------------------
// OPTIMIZED SEQUENTIAL UPDATE METHODS (3D / 3x3 Updates)
// ---------------------------------------------------------
/**
 * @brief Sequential measurement update performing Kalman correction.
 * @param measurement Observed 3D sensor reading in body frame.
 * @param reference Expected 3D reference vector in world frame.
 * @param R_meas 3x3 measurement noise covariance matrix.
 */
void MEKF::updateMeasurement(const Vector3f& measurement,
                             const Vector3f& reference,
                             const Matrix3f& R_meas) {
                             
    // Build rotational mapping from Body to Reference frame
    Matrix3f R_body_to_ref;
    float w = _quat(0), x = _quat(1), y = _quat(2), z = _quat(3);
    R_body_to_ref << 1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w,
                         2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w,
                         2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y;

    // Estimate what the measurement SHOULD be in the body frame (Transpose maps Ref -> Body)
    Vector3f v_hat = R_body_to_ref.transpose() * reference;

    // Innovation Residual
    Vector3f r = measurement - v_hat;

    // Jacobian Matrix (Only mapping orientation block, 3x6)
    Matrix<float, 3, 6> H = Matrix<float, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = skewSymmetric(v_hat);

    // Kalman Gain (S is only a 3x3 inversion, which Eigen resolves analytically)
    Matrix3f S = H * _cov * H.transpose() + R_meas;
    Matrix<float, 6, 3> K = _cov * H.transpose() * S.inverse();

    // Compute Error States
    Matrix<float, 6, 1> delta_x = K * r;

    // Apply Orientation Correction (Multiplicative - local frame right-multiplication)
    Vector3f delta_theta = delta_x.segment<3>(0);
    float theta_mag = delta_theta.norm();
    if (theta_mag > 1e-5f) {
        float half_theta = 0.5f * theta_mag;
        float s = sin(half_theta) / theta_mag;
        Vector4f dq;
        dq << cos(half_theta), delta_theta(0)*s, delta_theta(1)*s, delta_theta(2)*s;
        _quat = quatMult(_quat, dq);
        _quat.normalize();
    }

    // Apply Bias Correction (Additive)
    _bias += delta_x.segment<3>(3);

    // --- HIGH-SPEED COVARIANCE UPDATE ---
    Matrix<float, 6, 6> I_KH = Matrix<float, 6, 6>::Identity() - K * H;
    
    // Use .noalias() to strictly block Eigen's hidden memory allocation overhead
    Matrix<float, 6, 6> next_cov;
    next_cov.noalias() = I_KH * _cov; 
    
    // Force strict symmetry to emulate Joseph Form stability at a fraction of the CPU cost
    _cov = 0.5f * (next_cov + next_cov.transpose().eval());
}

/**
 * @brief Updates heading from magnetometer measurements.
 * @param mag_measure Measured magnetometer 3D vector.
 * @param mag_ref World magnetic reference 3D vector.
 * @param r_yaw Scalar yaw measurement noise variance.
 */
void MEKF::updateMagnetometerYaw(const Vector3f& mag_measure,
                                 const Vector3f& mag_ref,
                                 float r_yaw) {
    
    if (mag_measure.squaredNorm() < 1e-6f) return;

    // Expand the scalar yaw variance into a full 3x3 diagonal covariance matrix
    Matrix3f R_meas;
    R_meas << r_yaw, 0.0f,  0.0f,
              0.0f,  r_yaw, 0.0f,
              0.0f,  0.0f,  r_yaw;

    // Pass directly to the optimized 3D measurement update
    updateMeasurement(mag_measure.normalized(), mag_ref.normalized(), R_meas);
}

/**
 * @brief Main sequential MEKF filter step for IAttitudeFilter interface.
 * @param dt Delta time step in seconds.
 * @param ax Accelerometer X in g.
 * @param ay Accelerometer Y in g.
 * @param az Accelerometer Z in g.
 * @param gx Gyroscope X in rad/s.
 * @param gy Gyroscope Y in rad/s.
 * @param gz Gyroscope Z in rad/s.
 * @param mx Magnetometer X in uT.
 * @param my Magnetometer Y in uT.
 * @param mz Magnetometer Z in uT.
 * @param ignoreAccel Flag to skip accelerometer update.
 */
void MEKF::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {
    Vector3f gyro(gx, gy, gz);
    Vector3f accel(ax, ay, az);
    Vector3f mag(mx, my, mz);

    // 1. Independent Time Update
    predict(gyro, dt);

    // 2. Independent Gravity Update (3x3)
    if (!ignoreAccel && accel.norm() > 0.1f) {
        accel.normalize();
        Vector3f gravity_ref(0.0f, 0.0f, 1.0f);
        Matrix3f R_accel = Matrix3f::Identity() * _mekf_r_accel;
        updateMeasurement(accel, gravity_ref, R_accel);
    }

    // 3. Independent Compass Update (3x3)
    if (_useMagnetometer && mag.norm() > 1.0f) {
        Vector3f mag_ref(_mag_ref_x, _mag_ref_y, _mag_ref_z);
        updateMagnetometerYaw(mag, mag_ref, _mekf_r_mag);
    }
}
