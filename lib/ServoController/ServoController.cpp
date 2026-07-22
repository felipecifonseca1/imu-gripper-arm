#include "ServoController.h"

/**
 * @brief Constructs ServoController with default configuration values.
 */
ServoController::ServoController() 
    : _currentRoll(0.0f), _currentPitch(0.0f), _currentYaw(0.0f), 
      _alphaRoll(0.15f), _alphaPitch(0.15f), _alphaYaw(0.2f), 
      _deadbandRoll(0.5f), _deadbandPitch(0.5f), _deadbandYaw(0.5f),
      _maxSlewRoll(180.0f), _maxSlewPitch(180.0f), _maxSlewYaw(180.0f),
      _minImuRoll(-25.0f), _maxImuRoll(-6.0f),
      _minImuPitch(-90.0f), _maxImuPitch(90.0f),
      _minImuYaw(-90.0f), _maxImuYaw(90.0f),
      _lastUpdateMicros(0) {
}

/**
 * @brief Initializes PWM timers and attaches servo output pins.
 * @param gripperPin GPIO pin attached to gripper servo.
 * @param elevationPin GPIO pin attached to elevation servo.
 * @param yawPin GPIO pin attached to yaw servo.
 */
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

    _lastUpdateMicros = micros();
}

/**
 * @brief Sets filter alpha coefficients for exponential low-pass filtering.
 * @param alphaRoll Alpha coefficient for roll.
 * @param alphaPitch Alpha coefficient for pitch.
 * @param alphaYaw Alpha coefficient for yaw.
 */
void ServoController::setFilterAlphas(float alphaRoll, float alphaPitch, float alphaYaw) {
    _alphaRoll = constrain(alphaRoll, 0.0f, 1.0f);
    _alphaPitch = constrain(alphaPitch, 0.0f, 1.0f);
    _alphaYaw = constrain(alphaYaw, 0.0f, 1.0f);
}

/**
 * @brief Sets deadband thresholds for control axes.
 * @param deadbandRoll Deadband angle for roll.
 * @param deadbandPitch Deadband angle for pitch.
 * @param deadbandYaw Deadband angle for yaw.
 */
void ServoController::setDeadbands(float deadbandRoll, float deadbandPitch, float deadbandYaw) {
    _deadbandRoll = deadbandRoll;
    _deadbandPitch = deadbandPitch;
    _deadbandYaw = deadbandYaw;
}

/**
 * @brief Sets slew rate limits for angular speed.
 * @param maxDegPerSecRoll Maximum deg/s limit for roll.
 * @param maxDegPerSecPitch Maximum deg/s limit for pitch.
 * @param maxDegPerSecYaw Maximum deg/s limit for yaw.
 */
void ServoController::setSlewRateLimits(float maxDegPerSecRoll, float maxDegPerSecPitch, float maxDegPerSecYaw) {
    _maxSlewRoll = maxDegPerSecRoll;
    _maxSlewPitch = maxDegPerSecPitch;
    _maxSlewYaw = maxDegPerSecYaw;
}

/**
 * @brief Sets mapping range for IMU roll to servo position.
 * @param minImuRoll Minimum IMU roll angle.
 * @param maxImuRoll Maximum IMU roll angle.
 */
void ServoController::setRollRange(float minImuRoll, float maxImuRoll) {
    _minImuRoll = minImuRoll;
    _maxImuRoll = maxImuRoll;
}

/**
 * @brief Sets mapping range for IMU pitch to servo position.
 * @param minImuPitch Minimum IMU pitch angle.
 * @param maxImuPitch Maximum IMU pitch angle.
 */
void ServoController::setPitchRange(float minImuPitch, float maxImuPitch) {
    _minImuPitch = minImuPitch;
    _maxImuPitch = maxImuPitch;
}

/**
 * @brief Sets mapping range for IMU yaw to servo position.
 * @param minImuYaw Minimum IMU yaw angle.
 * @param maxImuYaw Maximum IMU yaw angle.
 */
void ServoController::setYawRange(float minImuYaw, float maxImuYaw) {
    _minImuYaw = minImuYaw;
    _maxImuYaw = maxImuYaw;
}

/**
 * @brief Main servo control update loop applying filtering, deadbands, and slew limits.
 * @param targetRoll Target roll orientation in degrees.
 * @param targetPitch Target pitch orientation in degrees.
 * @param targetYaw Target yaw orientation in degrees.
 */
void ServoController::update(float targetRoll, float targetPitch, float targetYaw) {
    unsigned long now = micros();
    float dt = (now - _lastUpdateMicros) / 1000000.0f;
    _lastUpdateMicros = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f; // Fallback for first frame or overflow

    // 0. Euler Wrapping Guard
    const float WRAP_THRESHOLD = 120.0f;
    if (abs(targetRoll) > WRAP_THRESHOLD)  targetRoll  = _currentRoll;
    if (abs(targetPitch) > WRAP_THRESHOLD) targetPitch = _currentPitch;
    if (abs(targetYaw) > WRAP_THRESHOLD)   targetYaw   = _currentYaw;

    // 1. Continuous Deadband Filter: Subtract deadband offset to avoid step discontinuities
    auto applyContinuousDeadband = [](float current, float target, float deadband) -> float {
        float diff = target - current;
        if (abs(diff) <= deadband) return current;
        return (diff > 0.0f) ? target - deadband : target + deadband;
    };

    float dbTargetRoll  = applyContinuousDeadband(_currentRoll, targetRoll, _deadbandRoll);
    float dbTargetPitch = applyContinuousDeadband(_currentPitch, targetPitch, _deadbandPitch);
    float dbTargetYaw   = applyContinuousDeadband(_currentYaw, targetYaw, _deadbandYaw);

    // 2. Exponential Moving Average (Low-Pass Filter)
    float lpfRoll  = _alphaRoll  * dbTargetRoll  + (1.0f - _alphaRoll)  * _currentRoll;
    float lpfPitch = _alphaPitch * dbTargetPitch + (1.0f - _alphaPitch) * _currentPitch;
    float lpfYaw   = _alphaYaw   * dbTargetYaw   + (1.0f - _alphaYaw)   * _currentYaw;

    // 3. Slew-Rate Limiter (Max angular velocity limit in deg/s)
    auto applySlewRate = [dt](float current, float desired, float maxDegPerSec) -> float {
        float maxChange = maxDegPerSec * dt;
        float diff = desired - current;
        if (diff > maxChange) return current + maxChange;
        if (diff < -maxChange) return current - maxChange;
        return desired;
    };

    _currentRoll  = applySlewRate(_currentRoll, lpfRoll, _maxSlewRoll);
    _currentPitch = applySlewRate(_currentPitch, lpfPitch, _maxSlewPitch);
    _currentYaw   = applySlewRate(_currentYaw, lpfYaw, _maxSlewYaw);

    // Helper lambda for float mapping
    auto mapFloat = [](float x, float in_min, float in_max, float out_min, float out_max) -> float {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };

    // Map angles directly to high-resolution microseconds (500us - 2500us is 0-180 deg)
    // Gripper (Roll):     Mapped from [_minImuRoll, _maxImuRoll] to [500us, 1500us] (0 - 90 deg Servo)
    // Elevation (Pitch):  Mapped from [_minImuPitch, _maxImuPitch] to [500us, 2500us] (0 - 180 deg Servo)
    // Yaw:                Mapped from [_minImuYaw, _maxImuYaw] to [500us, 2500us] (0 - 180 deg Servo)
    int gripperUs = (int)mapFloat(_currentRoll, _minImuRoll, _maxImuRoll, 500.0f, 1500.0f);
    int elevationUs = (int)mapFloat(_currentPitch, _minImuPitch, _maxImuPitch, 500.0f, 2500.0f);
    int yawUs = (int)mapFloat(_currentYaw, _minImuYaw, _maxImuYaw, 500.0f, 2500.0f);

    // Constrain to safe mechanical PWM limits
    gripperUs = constrain(gripperUs, 500, 1500);       // Gripper: 0-90 deg only
    elevationUs = constrain(elevationUs, 500, 2500);
    yawUs = constrain(yawUs, 500, 2500);

    _gripperServo.writeMicroseconds(gripperUs);
    _elevationServo.writeMicroseconds(elevationUs);
    _yawServo.writeMicroseconds(yawUs);
}

/**
 * @brief Reads current gripper servo command angle.
 * @return Servo position angle in degrees.
 */
int ServoController::getGripperAngle() {
    return _gripperServo.read();
}

/**
 * @brief Reads current elevation servo command angle.
 * @return Servo position angle in degrees.
 */
int ServoController::getElevationAngle() {
    return _elevationServo.read();
}

/**
 * @brief Reads current yaw servo command angle.
 * @return Servo position angle in degrees.
 */
int ServoController::getYawAngle() {
    return _yawServo.read();
}
