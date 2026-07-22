#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <math.h>
#include <Arduino.h>

class MadgwickFilter : public IAttitudeFilter {
public:
    /**
     * @brief Constructs MadgwickFilter instance.
     */
    MadgwickFilter();

    /**
     * @brief Main Madgwick orientation gradient descent update step.
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
     * @param ignoreAccel Flag to suppress accelerometer gradient step.
     */
    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;

    /**
     * @brief Gets current estimated unit quaternion.
     * @param w Output scalar w reference.
     * @param x Output vector x reference.
     * @param y Output vector y reference.
     * @param z Output vector z reference.
     */
    void getQuaternion(float& w, float& x, float& y, float& z) const override;

    /**
     * @brief Sets unit quaternion state and normalizes.
     * @param w Quaternion scalar w component.
     * @param x Quaternion vector x component.
     * @param y Quaternion vector y component.
     * @param z Quaternion vector z component.
     */
    void setQuaternion(float w, float x, float y, float z) override;

    /**
     * @brief Resets quaternion to identity and biases to zero.
     */
    void reset() override;

    /**
     * @brief Sets filter beta gain based on estimated gyro error rate.
     * @param errorDegPerSec Estimated gyroscope drift error in deg/s.
     */
    void setFilterBeta(float errorDegPerSec);

    /**
     * @brief Enables or disables zeta drift learning.
     * @param enabled True to enable drift learning.
     */
    void setDriftLearning(bool enabled);

    /**
     * @brief Sets magnetometer fusion weight factor.
     * @param weight Weight factor between 0.0 and 1.0.
     */
    void setMagnetometerWeight(float weight);

private:
    float _q[4];
    
    float _beta;
    float _zeta;
    bool _useZeta;
    float _w_bx, _w_by, _w_bz;
    float _magWeight;
};

#endif // MADGWICK_FILTER_H
