#include "AttitudeEstimator.h"

ESKFFilter::ESKFFilter() {
    reset();
}

void ESKFFilter::reset() {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
}

void ESKFFilter::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = _q[0]; x = _q[1]; y = _q[2]; z = _q[3];
}

void ESKFFilter::setQuaternion(float w, float x, float y, float z) {
    _q[0] = w; _q[1] = x; _q[2] = y; _q[3] = z;
    float recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}

void ESKFFilter::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) {
    // TODO: Implement Error-State Kalman Filter
}
