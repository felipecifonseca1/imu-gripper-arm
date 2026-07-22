#ifndef ESKF_FILTER_H
#define ESKF_FILTER_H

#include <Arduino.h>
#include <ArduinoEigen.h>

class ESKFFilter : public IAttitudeFilter {
public:
    /**
     * @brief Constructs ESKFFilter instance.
     */
    ESKFFilter();

    /**
     * @brief High-frequency state prediction phase using gyroscope integration.
     * @param gyro Gyroscope measurement vector in rad/s.
     * @param dt Integration time step in seconds.
     */
    void predict(const Eigen::Vector3f& gyro, float dt);

    /**
     * @brief Accelerometer measurement correction step.
     * @param accel Accelerometer measurement vector in g.
     * @param dt Integration time step in seconds.
     */
    void updateAccelerometer(const Eigen::Vector3f& accel, float dt);

    /**
     * @brief Magnetometer measurement correction step.
     * @param mag Magnetometer measurement vector in uT.
     * @param dt Integration time step in seconds.
     */
    void updateMagnetometer(const Eigen::Vector3f& mag, float dt);

    /**
     * @brief Sequential multi-sensor update override for IAttitudeFilter interface.
     * @param dt Integration time step in seconds.
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
    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;

    /**
     * @brief Gets current nominal quaternion.
     * @param w Output scalar w reference.
     * @param x Output vector x reference.
     * @param y Output vector y reference.
     * @param z Output vector z reference.
     */
    void getQuaternion(float& w, float& x, float& y, float& z) const override;

    /**
     * @brief Sets nominal quaternion state.
     * @param w Quaternion scalar w component.
     * @param x Quaternion vector x component.
     * @param y Quaternion vector y component.
     * @param z Quaternion vector z component.
     */
    void setQuaternion(float w, float x, float y, float z) override;

    /**
     * @brief Resets ESKF filter states and covariance matrices.
     */
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
    /**
     * @brief Computes skew-symmetric matrix for 3D vector.
     * @param v Input 3D vector.
     * @return Skew-symmetric 3x3 matrix.
     */
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v);

    /**
     * @brief Converts rotation vector to delta quaternion.
     * @param r Rotation vector.
     * @return Delta quaternion representation.
     */
    Eigen::Quaternionf fromRotationVector(const Eigen::Vector3f& r);

    /**
     * @brief Extracts gravity direction vector from rotation matrix.
     * @param R Rotation matrix.
     * @return Unit gravity vector.
     */
    Eigen::Vector3f rotMatToGravity(const Eigen::Matrix3f& R);

    /**
     * @brief Extracts magnetic vector direction from rotation matrix.
     * @param R Rotation matrix.
     * @return Unit magnetic field vector.
     */
    Eigen::Vector3f rotMatToMagnetic(const Eigen::Matrix3f& R);

    /**
     * @brief Computes initial orientation quaternion from accelerometer and magnetometer vectors.
     * @param a Accelerometer vector.
     * @param m Magnetometer vector.
     * @return Unit quaternion representation of initial orientation.
     */
    Eigen::Quaternionf ecompass(const Eigen::Vector3f& a, const Eigen::Vector3f& m);

    /**
     * @brief Limits vector norm to maximum threshold.
     * @param v Input vector.
     * @param max_norm Maximum allowable vector magnitude.
     * @return Norm-limited 3D vector.
     */
    Eigen::Vector3f limitVectorNorm(const Eigen::Vector3f& v, float max_norm);

    /**
     * @brief Builds measurement Jacobian sub-matrix for vector.
     * @param v Input 3D vector.
     * @return 3x3 Jacobian sub-matrix.
     */
    Eigen::Matrix3f buildHPart(const Eigen::Vector3f& v);
};

#endif // ESKF_FILTER_H
