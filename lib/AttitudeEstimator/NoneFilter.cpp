#include "AttitudeEstimator.h"

NoneFilter::NoneFilter() {
    reset();
}

void NoneFilter::reset() {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
}

void NoneFilter::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = _q[0]; x = _q[1]; y = _q[2]; z = _q[3];
}

void NoneFilter::setQuaternion(float w, float x, float y, float z) {
    _q[0] = w; _q[1] = x; _q[2] = y; _q[3] = z;
    float recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}

void NoneFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {
    float q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];
    _q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    _q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    _q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    _q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;
    
    float recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}
