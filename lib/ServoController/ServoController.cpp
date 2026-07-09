#include "ServoController.h"

ServoController::ServoController() 
    : _currentRoll(0.0), _currentPitch(0.0), _currentYaw(0.0), _alpha(0.2), _deadband(2.0) {
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

void ServoController::setFilterAlpha(float alpha) {
    _alpha = constrain(alpha, 0.0f, 1.0f);
}

void ServoController::setDeadband(float deadband) {
    _deadband = deadband;
}

void ServoController::update(float targetRoll, float targetPitch, float targetYaw) {
    // Apply Deadband and Lowpass to Roll (Gripper)
    if (abs(targetRoll - _currentRoll) > _deadband) {
        _currentRoll = _alpha * targetRoll + (1.0 - _alpha) * _currentRoll;
    }

    // Apply Deadband and Lowpass to Pitch (Elevation)
    if (abs(targetPitch - _currentPitch) > _deadband) {
        _currentPitch = _alpha * targetPitch + (1.0 - _alpha) * _currentPitch;
    }

    // Apply Deadband and Lowpass to Yaw
    if (abs(targetYaw - _currentYaw) > _deadband) {
        _currentYaw = _alpha * targetYaw + (1.0 - _alpha) * _currentYaw;
    }

    // Map angles to Servo PWM (0-180 degrees)
    // Assuming IMU gives roll/pitch/yaw from -90 to 90 degrees for this mapping
    int gripperAngle = map(_currentRoll, -90, 90, 0, 180);
    int elevationAngle = map(_currentPitch, -90, 90, 0, 180);
    int yawAngle = map(_currentYaw, -90, 90, 0, 180);

    // Constrain angles to safe ranges to avoid hitting mechanical limits
    gripperAngle = constrain(gripperAngle, 10, 170);
    elevationAngle = constrain(elevationAngle, 10, 170);
    yawAngle = constrain(yawAngle, 10, 170);

    _gripperServo.write(gripperAngle);
    _elevationServo.write(elevationAngle);
    _yawServo.write(yawAngle);
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
