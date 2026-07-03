#include "AttitudeEstimator.h"

MadgwickFilter::MadgwickFilter() {
    _beta = sqrt(3.0f / 4.0f) * (PI * (1.0f / 180.0f));
    _zeta = sqrt(3.0f / 4.0f) * (PI * (1.0f / 180.0f));
    _useZeta = false;
    _w_bx = 0.0f; _w_by = 0.0f; _w_bz = 0.0f;
    _magWeight = 0.05f;
    reset();
}

void MadgwickFilter::reset() {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
    _w_bx = 0.0f; _w_by = 0.0f; _w_bz = 0.0f;
}

void MadgwickFilter::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = _q[0]; x = _q[1]; y = _q[2]; z = _q[3];
}

void MadgwickFilter::setQuaternion(float w, float x, float y, float z) {
    _q[0] = w; _q[1] = x; _q[2] = y; _q[3] = z;
    float recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}

void MadgwickFilter::setFilterBeta(float errorDegPerSec) {
    float gyroErr = PI * (errorDegPerSec / 180.0f);
    _beta = sqrt(3.0f / 4.0f) * gyroErr;
    _zeta = _beta; 
}

void MadgwickFilter::setDriftLearning(bool enabled) {
    _useZeta = enabled;
}

void MadgwickFilter::setMagnetometerWeight(float weight) {
    if (weight < 0.0f) weight = 0.0f;
    if (weight > 1.0f) weight = 1.0f;
    _magWeight = weight;
}

void MadgwickFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {
    double q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double hx, hy;
    double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    if (ignoreAccel) { ax = 0; ay = 0; az = 0; }

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);


    double a_norm_sq = ax * ax + ay * ay + az * az;
    double m_norm_sq = mx * mx + my * my + mz * mz;

    s0 = 0; s1 = 0; s2 = 0; s3 = 0;

    // Common term pre-computation
    _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
    q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

    // 1. Accelerometer Component
    if (a_norm_sq > 0.0f) {
        recipNorm = 1.0 / sqrt(a_norm_sq);
        ax *= (float)recipNorm; ay *= (float)recipNorm; az *= (float)recipNorm;

        s0 += -_2q2 * (2.0f * (q1q3 - q0q2) - ax) + _2q1 * (2.0f * (q0q1 + q2q3) - ay);
        s1 += _2q3 * (2.0f * (q1q3 - q0q2) - ax) + _2q0 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s2 += -_2q0 * (2.0f * (q1q3 - q0q2) - ax) + _2q3 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s3 += _2q1 * (2.0f * (q1q3 - q0q2) - ax) + _2q2 * (2.0f * (q0q1 + q2q3) - ay);
    }

    // 2. Magnetometer Component
    if (m_norm_sq > 0.0f) {
        recipNorm = 1.0 / sqrt(m_norm_sq);
        mx *= (float)recipNorm; my *= (float)recipNorm; mz *= (float)recipNorm;

        _2q0mx = 2.0f * q0 * mx; _2q0my = 2.0f * q0 * my; _2q0mz = 2.0f * q0 * mz; _2q1mx = 2.0f * q1 * mx;
        _2q0q2 = 2.0f * q0 * q2; _2q2q3 = 2.0f * q2 * q3;

        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;

        s0 += _magWeight * (-_2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz));
        s1 += _magWeight * (_2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz));
        s2 += _magWeight * ((-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz));
        s3 += _magWeight * ((-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz));
    }

    double s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (s_norm_sq > 0.0) {
        recipNorm = 1.0 / sqrt(s_norm_sq);
        s0 *= (float)recipNorm; s1 *= (float)recipNorm; s2 *= (float)recipNorm; s3 *= (float)recipNorm;
    }

    if (_useZeta) {
        double s_w_x = 2.0f * (q0 * s1 - q1 * s0 - q2 * s3 + q3 * s2);
        double s_w_y = 2.0f * (q0 * s2 + q1 * s3 - q2 * s0 - q3 * s1);
        double s_w_z = 2.0f * (q0 * s3 - q1 * s2 + q2 * s1 - q3 * s0);

        _w_bx += _zeta * s_w_x * dt;
        _w_by += _zeta * s_w_y * dt;
        _w_bz += _zeta * s_w_z * dt;
    }

    gx -= _w_bx; gy -= _w_by; gz -= _w_bz; 

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - _beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - _beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - _beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - _beta * s3;

    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    _q[0] = q0 * recipNorm; _q[1] = q1 * recipNorm; _q[2] = q2 * recipNorm; _q[3] = q3 * recipNorm;
}
