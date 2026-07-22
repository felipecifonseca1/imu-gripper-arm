#ifndef NONE_FILTER_H
#define NONE_FILTER_H

#include <math.h>

class NoneFilter : public IAttitudeFilter {
public:
    /**
     * @brief Constructs NoneFilter instance.
     */
    NoneFilter();

    /**
     * @brief Pure gyro integration update step (no sensor correction).
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
     * @param ignoreAccel Flag for accelerometer ignore.
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
     * @brief Resets quaternion state to identity.
     */
    void reset() override;

private:
    float _q[4];
};

#endif // NONE_FILTER_H
