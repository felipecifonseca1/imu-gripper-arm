#include "AttitudeEstimator.h"

/**
 * @brief Constructs NoneFilter object and resets quaternion to identity.
 */
NoneFilter::NoneFilter() {
    reset();
}

/**
 * @brief Resets quaternion array to identity.
 */
void NoneFilter::reset() {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
}

/**
 * @brief Gets estimated unit quaternion components.
 * @param w Output w scalar reference.
 * @param x Output x vector reference.
 * @param y Output y vector reference.
 * @param z Output z vector reference.
 */
void NoneFilter::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = _q[0]; x = _q[1]; y = _q[2]; z = _q[3];
}

/**
 * @brief Sets unit quaternion state and normalizes.
 * @param w Quaternion scalar w component.
 * @param x Quaternion vector x component.
 * @param y Quaternion vector y component.
 * @param z Quaternion vector z component.
 */
void NoneFilter::setQuaternion(float w, float x, float y, float z) {
    _q[0] = w; _q[1] = x; _q[2] = y; _q[3] = z;
    float recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}

/**
 * @brief Performs un-corrected kinematic integration using gyroscope rate measurements.
 * @param dt Integration time step in seconds.
 * @param ax Accelerometer X (unused).
 * @param ay Accelerometer Y (unused).
 * @param az Accelerometer Z (unused).
 * @param gx Gyroscope X angular rate in rad/s.
 * @param gy Gyroscope Y angular rate in rad/s.
 * @param gz Gyroscope Z angular rate in rad/s.
 * @param mx Magnetometer X (unused).
 * @param my Magnetometer Y (unused).
 * @param mz Magnetometer Z (unused).
 * @param ignoreAccel Flag for ignoring accel (unused).
 */
void NoneFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {
    float q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];
    _q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    _q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    _q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    _q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;
    
    float recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}
