#include "AttitudeEstimator.h"

/**
 * @brief Constructs MahonyFilter object and initializes default gains.
 */
MahonyFilter::MahonyFilter() {
    _Kp = 30.0f;
    _Ki = 0.0f;
    _ix = 0.0f; _iy = 0.0f; _iz = 0.0f;
    _magWeight = 0.05f;
    reset();
}

/**
 * @brief Resets orientation quaternion to identity and integral errors to zero.
 */
void MahonyFilter::reset() {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
    _ix = 0.0f; _iy = 0.0f; _iz = 0.0f;
}

/**
 * @brief Gets estimated unit quaternion components.
 * @param w Output w scalar reference.
 * @param x Output x vector reference.
 * @param y Output y vector reference.
 * @param z Output z vector reference.
 */
void MahonyFilter::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = _q[0]; x = _q[1]; y = _q[2]; z = _q[3];
}

/**
 * @brief Sets unit quaternion state and normalizes.
 * @param w Quaternion scalar w component.
 * @param x Quaternion vector x component.
 * @param y Quaternion vector y component.
 * @param z Quaternion vector z component.
 */
void MahonyFilter::setQuaternion(float w, float x, float y, float z) {
    _q[0] = w; _q[1] = x; _q[2] = y; _q[3] = z;
    float recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}

/**
 * @brief Sets magnetometer weighting factor.
 * @param weight Weight value between 0.0 and 1.0.
 */
void MahonyFilter::setMagnetometerWeight(float weight) {
    if (weight < 0.0f) weight = 0.0f;
    if (weight > 1.0f) weight = 1.0f;
    _magWeight = weight;
}

/**
 * @brief Sets proportional (Kp) and integral (Ki) feedback gains.
 * @param kp Proportional feedback gain.
 * @param ki Integral feedback gain.
 */
void MahonyFilter::setGains(float kp, float ki) {
    _Kp = kp;
    _Ki = ki;
}

/**
 * @brief Main Mahony orientation PI feedback algorithm update step.
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
void MahonyFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {
    if (ignoreAccel) { ax = 0; ay = 0; az = 0; }
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;  
    float qa, qb, qc;
    float tmp;

    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0f) {
        recipNorm = 1.0f / sqrt(tmp);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        vx = _q[1] * _q[3] - _q[0] * _q[2];
        vy = _q[0] * _q[1] + _q[2] * _q[3];
        vz = _q[0] * _q[0] - 0.5f + _q[3] * _q[3];

        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        double m_norm = mx * mx + my * my + mz * mz;
        if (m_norm > 0.0) {
            recipNorm = 1.0 / sqrt(m_norm);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            float hx = mx * _q[0] * _q[0] - 2.0f * _q[0] * my * _q[3] + 2.0f * _q[0] * mz * _q[2] + mx * _q[1] * _q[1] + 2.0f * _q[1] * my * _q[2] + 2.0f * _q[1] * mz * _q[3] - mx * _q[2] * _q[2] - mx * _q[3] * _q[3];
            float hy = 2.0f * _q[0] * mx * _q[3] + my * _q[0] * _q[0] - 2.0f * _q[0] * mz * _q[1] + 2.0f * _q[1] * mx * _q[2] - my * _q[1] * _q[1] + my * _q[2] * _q[2] + 2.0f * _q[2] * mz * _q[3] - my * _q[3] * _q[3];
            float bx = sqrt(hx * hx + hy * hy);
            float bz = -2.0f * _q[0] * mx * _q[2] + 2.0f * _q[0] * my * _q[1] + mz * _q[0] * _q[0] + 2.0f * _q[1] * mx * _q[3] - mz * _q[1] * _q[1] + 2.0f * _q[2] * my * _q[3] - mz * _q[2] * _q[2] + mz * _q[3] * _q[3];

            float wx = 2.0f * bx * (0.5f - _q[2] * _q[2] - _q[3] * _q[3]) + 2.0f * bz * (_q[1] * _q[3] - _q[0] * _q[2]);
            float wy = 2.0f * bx * (_q[1] * _q[2] - _q[0] * _q[3]) + 2.0f * bz * (_q[0] * _q[1] + _q[2] * _q[3]);
            float wz = 2.0f * bx * (_q[0] * _q[2] + _q[1] * _q[3]) + 2.0f * bz * (0.5f - _q[1] * _q[1] - _q[2] * _q[2]);

            ex += _magWeight * (my * wz - mz * wy);
            ey += _magWeight * (mz * wx - mx * wz);
            ez += _magWeight * (mx * wy - my * wx);
        }

        if (_Ki > 0.0f) {
            _ix += _Ki * ex * dt;
            _iy += _Ki * ey * dt;
            _iz += _Ki * ez * dt;
            gx += _ix; gy += _iy; gz += _iz;
        }

        gx += _Kp * ex; gy += _Kp * ey; gz += _Kp * ez;
    }

    float halfDt = 0.5f * dt;
    gx *= halfDt; gy *= halfDt; gz *= halfDt;
    qa = _q[0]; qb = _q[1]; qc = _q[2];
    _q[0] += (-qb * gx - qc * gy - _q[3] * gz);
    _q[1] += (qa * gx + qc * gz - _q[3] * gy);
    _q[2] += (qa * gy - qb * gz + _q[3] * gx);
    _q[3] += (qa * gz + qb * gy - qc * gx);

    recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}
