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
    MEKF();
    ~MEKF() override = default;

    void setProcessNoise(float q_diagonal);

    // The high-frequency prediction step
    void predict(const Vector3f& gyro, float dt);

    // The sequential correction steps
    void updateMeasurement(const Vector3f& measurement,
                           const Vector3f& reference,
                           const Matrix3f& R_meas);

    void updateMagnetometerYaw(const Vector3f& mag_measure,
                               const Vector3f& mag_ref,
                               float r_yaw);

    Vector3f getGyroBias() const { return _bias; }
    void resetState();

    // --- IAttitudeFilter overrides ---
    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;
    void getQuaternion(float& w, float& x, float& y, float& z) const override;
    void setQuaternion(float w, float x, float y, float z) override;
    void reset() override;

    // --- MEKF specific tuning ---
    void setMEKFTuning(float q_proc, float r_accel, float r_mag);
    void setUseMagnetometer(bool use);
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
    Matrix3f skewSymmetric(const Vector3f& v);
    void forceSymmetry();
    Vector4f quatMult(const Vector4f& q1, const Vector4f& q2);
};

#endif // MEKF_H