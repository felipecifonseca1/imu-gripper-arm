// #ifndef MEKF_H
// #define MEKF_H

// #include <Arduino.h>
// #include <ArduinoEigen.h>

// using namespace Eigen;

// /**
//  * @class MEKF
//  * @brief Multiplicative Extended Kalman Filter for attitude estimation.
//  *
//  * @details Estimates rocket orientation as a unit quaternion using gyroscope
//  *          measurements for prediction and accelerometer/magnetometer measurements
//  *          for correction.
//  */
// class MEKF : public IAttitudeFilter {
// public:
//     MEKF();
//     ~MEKF() override = default;

//     void init(const Matrix<float, 6, 6>& Q_in, const Matrix<float, 6, 6>& P0_in);
//     void setProcessNoise(float q_diagonal);

//     void predict(const Matrix<float, 3, 1>& gyro, float dt);

//     void updateMeasurement(const Matrix<float, 3, 1>& measurement,
//                            const Matrix<float, 3, 1>& reference,
//                            const Matrix<float, 3, 3>& R_meas);

//     void updateMagnetometerYaw(const Eigen::Matrix<float, 3, 1>& mag_measure,
//                                const Eigen::Matrix<float, 3, 1>& mag_ref,
//                                float r_yaw);

//     Matrix<float, 3, 1> getGyroBias() const { return _bias; }

//     void resetState();

//     // --- IAttitudeFilter overrides ---
//     void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;
//     void getQuaternion(float& w, float& x, float& y, float& z) const override;
//     void setQuaternion(float w, float x, float y, float z) override;
//     void reset() override;

//     // --- MEKF specific tuning from Estimator ---
//     void setMEKFTuning(float q_proc, float r_accel, float r_mag);
//     void setUseMagnetometer(bool use);
//     void setMagneticReference(float mx, float my, float mz);

// private:
//     // States
//     Matrix<float, 4, 1> _quat;    
//     Matrix<float, 3, 1> _bias;    

//     Matrix<float, 6, 6> _cov;  
//     Matrix<float, 6, 6> _proc; 

//     bool _useMagnetometer;
//     float _mekf_q_proc;
//     float _mekf_r_accel;
//     float _mekf_r_mag;
    
//     float _mag_ref_x;
//     float _mag_ref_y;
//     float _mag_ref_z;

//     Matrix<float, 3, 3> skewSymmetric(const Matrix<float, 3, 1>& v) const;
//     Matrix<float, 4, 1> quatMultiply(const Matrix<float, 4, 1>& q1, const Matrix<float, 4, 1>& q2) const;
//     Matrix<float, 3, 3> quatToRotationMatrix(const Matrix<float, 4, 1>& qv) const;
// };

// #endif // MEKF_H


#ifndef MEKF_H
#define MEKF_H

#include <Arduino.h>
#include <ArduinoEigen.h>

using namespace Eigen;

class MEKF : public IAttitudeFilter {
public:
    /**
     * @brief Constructs MEKF instance.
     */
    MEKF();

    /**
     * @brief Default virtual destructor for MEKF.
     */
    ~MEKF() override = default;

    /**
     * @brief Sets process noise diagonal parameter value.
     * @param q_diagonal Process noise covariance multiplier.
     */
    void setProcessNoise(float q_diagonal);

    /**
     * @brief High-frequency state and error covariance prediction step.
     * @param gyro Unbiased gyroscope reading in rad/s.
     * @param dt Integration time step in seconds.
     */
    void predict(const Vector3f& gyro, float dt);

    /**
     * @brief Sequential vector measurement correction update (e.g. Gravity or Magnetic).
     * @param measurement Observed 3D sensor vector in body frame.
     * @param reference Expected 3D reference vector in world frame.
     * @param R_meas Measurement noise covariance matrix (3x3).
     */
    void updateMeasurement(const Vector3f& measurement,
                           const Vector3f& reference,
                           const Matrix3f& R_meas);

    /**
     * @brief Magnetometer update wrapper mapping yaw observations.
     * @param mag_measure Measured magnetometer 3D vector.
     * @param mag_ref World magnetic reference 3D vector.
     * @param r_yaw Yaw measurement noise variance.
     */
    void updateMagnetometerYaw(const Vector3f& mag_measure,
                               const Vector3f& mag_ref,
                               float r_yaw);

    /**
     * @brief Gets current estimated gyroscope bias vector.
     * @return 3D gyro bias vector in rad/s.
     */
    Vector3f getGyroBias() const { return _bias; }

    /**
     * @brief Resets quaternion state to identity and bias to zero.
     */
    void resetState();

    // --- IAttitudeFilter overrides ---
    /**
     * @brief Main filter update step for IAttitudeFilter interface.
     * @param dt Time delta step in seconds.
     * @param ax Accelerometer X in g.
     * @param ay Accelerometer Y in g.
     * @param az Accelerometer Z in g.
     * @param gx Gyroscope X in rad/s.
     * @param gy Gyroscope Y in rad/s.
     * @param gz Gyroscope Z in rad/s.
     * @param mx Magnetometer X in uT.
     * @param my Magnetometer Y in uT.
     * @param mz Magnetometer Z in uT.
     * @param ignoreAccel Flag to ignore accelerometer correction.
     */
    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;

    /**
     * @brief Gets estimated unit quaternion components.
     * @param w Output scalar w reference.
     * @param x Output vector x reference.
     * @param y Output vector y reference.
     * @param z Output vector z reference.
     */
    void getQuaternion(float& w, float& x, float& y, float& z) const override;

    /**
     * @brief Sets unit quaternion state.
     * @param w Quaternion scalar w component.
     * @param x Quaternion vector x component.
     * @param y Quaternion vector y component.
     * @param z Quaternion vector z component.
     */
    void setQuaternion(float w, float x, float y, float z) override;

    /**
     * @brief Resets filter states and covariance.
     */
    void reset() override;

    // --- MEKF specific tuning ---
    /**
     * @brief Configures MEKF tuning parameters.
     * @param q_proc Process noise parameter.
     * @param r_accel Accelerometer measurement noise parameter.
     * @param r_mag Magnetometer measurement noise parameter.
     */
    void setMEKFTuning(float q_proc, float r_accel, float r_mag);

    /**
     * @brief Enables or disables magnetometer fusion in MEKF.
     * @param use True to use magnetometer.
     */
    void setUseMagnetometer(bool use);

    /**
     * @brief Sets magnetic reference vector in world frame.
     * @param mx World magnetic reference X component.
     * @param my World magnetic reference Y component.
     * @param mz World magnetic reference Z component.
     */
    void setMagneticReference(float mx, float my, float mz);

private:
    // States
    Vector4f _quat;    
    Vector3f _bias;    

    // Covariance Array
    Matrix<float, 6, 6> _cov;  
    
    // Process Noise separated into 3x3 Diagonal Blocks
    Matrix3f _Q_theta; 
    Matrix3f _Q_bias;  

    bool _useMagnetometer;
    float _mekf_q_proc;
    float _mekf_r_accel;
    float _mekf_r_mag;
    
    float _mag_ref_x, _mag_ref_y, _mag_ref_z;

    // Internal Helper Math Functions
    /**
     * @brief Computes skew-symmetric matrix for 3D vector.
     * @param v Input 3D vector.
     * @return Skew-symmetric 3x3 matrix.
     */
    Matrix3f skewSymmetric(const Vector3f& v);

    /**
     * @brief Forces error covariance matrix symmetry.
     */
    void forceSymmetry();

    /**
     * @brief Hamilton multiplication of two quaternions.
     * @param q1 First quaternion vector.
     * @param q2 Second quaternion vector.
     * @return Product quaternion vector.
     */
    Vector4f quatMult(const Vector4f& q1, const Vector4f& q2);
};

#endif // MEKF_H