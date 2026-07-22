#include "ServoController.h"

ServoController::ServoController() 
    : _currentRoll(0.0), _currentPitch(0.0), _currentYaw(0.0), 
      _lockedTargetRoll(0.0), _lockedTargetPitch(0.0), _lockedTargetYaw(0.0),
      _alphaRoll(0.05), _alphaPitch(0.05), _alphaYaw(0.05), _deadbandRoll(2.0), _deadbandPitch(2.0), _deadbandYaw(2.0) {
}

void ServoController::init(int gripperPin, int elevationPin, int yawPin) {
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    _gripperServo.setPeriodHertz(50);
    _gripperServo.attach(gripperPin, 500, 2500);

    _elevationServo.setPeriodHertz(50);
    _elevationServo.attach(elevationPin, 500, 2500);

    _yawServo.setPeriodHertz(50);
    _yawServo.attach(yawPin, 500, 2500);
}

void ServoController::setFilterAlphas(float alphaRoll, float alphaPitch, float alphaYaw) {
    _alphaRoll = constrain(alphaRoll, 0.0f, 1.0f);
    _alphaPitch = constrain(alphaPitch, 0.0f, 1.0f);
    _alphaYaw = constrain(alphaYaw, 0.0f, 1.0f);
}

void ServoController::setDeadbands(float deadbandRoll, float deadbandPitch, float deadbandYaw) {
    _deadbandRoll = deadbandRoll;
    _deadbandPitch = deadbandPitch;
    _deadbandYaw = deadbandYaw;
}

void ServoController::update(float targetRoll, float targetPitch, float targetYaw) {
    // 0. Euler Wrapping Guard: Reject values near the ±180° discontinuity.
    //    If a value is beyond ±120°, it's likely a wrapping artifact.
    //    Keep the last known good locked target instead.
    const float WRAP_THRESHOLD = 120.0f;
    if (abs(targetRoll) > WRAP_THRESHOLD)  targetRoll  = _lockedTargetRoll;
    if (abs(targetPitch) > WRAP_THRESHOLD) targetPitch = _lockedTargetPitch;
    if (abs(targetYaw) > WRAP_THRESHOLD)   targetYaw   = _lockedTargetYaw;

    // 1. Deadband Logic: Update the "locked" target only if the IMU moved significantly
    if (abs(targetRoll - _lockedTargetRoll) > _deadbandRoll) {
        _lockedTargetRoll = targetRoll;
    }
    if (abs(targetPitch - _lockedTargetPitch) > _deadbandPitch) {
        _lockedTargetPitch = targetPitch;
    }
    if (abs(targetYaw - _lockedTargetYaw) > _deadbandYaw) {
        _lockedTargetYaw = targetYaw;
    }

    // 2. Low-Pass Filter Logic: Smoothly slide the current position to the locked target
    _currentRoll = _alphaRoll * _lockedTargetRoll + (1.0 - _alphaRoll) * _currentRoll;
    _currentPitch = _alphaPitch * _lockedTargetPitch + (1.0 - _alphaPitch) * _currentPitch;
    _currentYaw = _alphaYaw * _lockedTargetYaw + (1.0 - _alphaYaw) * _currentYaw;

    // Helper lambda for float mapping
    auto mapFloat = [](float x, float in_min, float in_max, float out_min, float out_max) -> float {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };

    // Map angles directly to high-resolution microseconds (500us - 2500us is 0-180 deg)
    // Gripper (Roll):     -25 deg IMU = 0 deg Servo (closed), -6 deg IMU = 90 deg Servo (open)
    // Elevation (Pitch):  0 deg IMU = 90 deg Servo (center), ±90 deg = full range
    // Yaw:                0 deg IMU = 90 deg Servo (center), ±90 deg = full range
    int gripperUs = (int)mapFloat(_currentRoll, -25.0, -6.0, 500.0, 1500.0);
    int elevationUs = (int)mapFloat(_currentPitch, -90.0, 90.0, 500.0, 2500.0);
    int yawUs = (int)mapFloat(_currentYaw, -90.0, 90.0, 500.0, 2500.0);

    // Constrain to safe mechanical PWM limits
    gripperUs = constrain(gripperUs, 500, 1500);       // Gripper: 0-90 deg only
    elevationUs = constrain(elevationUs, 500, 2500);
    yawUs = constrain(yawUs, 500, 2500);

    _gripperServo.writeMicroseconds(gripperUs);
    _elevationServo.writeMicroseconds(elevationUs);
    _yawServo.writeMicroseconds(yawUs);
}

int ServoController::getGripperAngle() {
    return _gripperServo.read();
}

int ServoController::getElevationAngle() {
    return _elevationServo.read();
}

int ServoController::getYawAngle() {
    return _yawServo.read();
}
