#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include <math.h>

class MahonyFilter : public IAttitudeFilter {
public:
    /**
     * @brief Constructs MahonyFilter instance with default gains.
     */
    MahonyFilter();

    /**
     * @brief Main Mahony orientation PI feedback update step.
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
     * @param ignoreAccel Flag to suppress accelerometer feedback.
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
     * @brief Resets quaternion to identity and integral error accumulators to zero.
     */
    void reset() override;

    /**
     * @brief Sets magnetometer weighting coefficient.
     * @param weight Weight factor between 0.0 and 1.0.
     */
    void setMagnetometerWeight(float weight);

    /**
     * @brief Sets proportional (Kp) and integral (Ki) feedback gains.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     */
    void setGains(float kp, float ki);

private:
    float _q[4];
    
    float _Kp;
    float _Ki;
    float _ix, _iy, _iz; // Integral feedback terms
    float _magWeight;
};

#endif // MAHONY_FILTER_H
