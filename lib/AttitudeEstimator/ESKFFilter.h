#ifndef ESKF_FILTER_H
#define ESKF_FILTER_H

#include <Arduino.h>
#include <ArduinoEigen.h>

class ESKFFilter : public IAttitudeFilter {
public:
    ESKFFilter();

    // High-frequency state propagation (Call this whenever new Gyro data arrives)
    void predict(const Eigen::Vector3f& gyro, float dt);

    // Independent multi-rate corrections (Call these whenever specific data packages land)
    void updateAccelerometer(const Eigen::Vector3f& accel, float dt);
    void updateMagnetometer(const Eigen::Vector3f& mag, float dt);

    // Unified wrapper override (calls the sequential steps internally)
    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;
    void getQuaternion(float& w, float& x, float& y, float& z) const override;
    void setQuaternion(float w, float x, float y, float z) override;
    void reset() override;

private:
    // Nominal state
    Eigen::Quaternionf _q_nom;
    Eigen::Vector3f _b_g;     // Gyro bias (rad/s)
    Eigen::Vector3f _a_lin;   // Linear acceleration (m/s^2)

    // Error covariance (9x9)
    Eigen::Matrix<float, 9, 9> _P_cov;

    // Filter status
    bool _is_first_sample;

    // Helper functions
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v);
    Eigen::Quaternionf fromRotationVector(const Eigen::Vector3f& r);
    Eigen::Vector3f rotMatToGravity(const Eigen::Matrix3f& R);
    Eigen::Vector3f rotMatToMagnetic(const Eigen::Matrix3f& R);
    Eigen::Quaternionf ecompass(const Eigen::Vector3f& a, const Eigen::Vector3f& m);
    Eigen::Vector3f limitVectorNorm(const Eigen::Vector3f& v, float max_norm);
    Eigen::Matrix3f buildHPart(const Eigen::Vector3f& v);
};

#endif // ESKF_FILTER_H
