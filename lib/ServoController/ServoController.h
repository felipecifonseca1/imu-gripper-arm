#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoController {
public:
    /**
     * @brief Constructs ServoController with default gains, limits, and mappings.
     */
    ServoController();

    /**
     * @brief Initializes PWM timers and attaches servo output pins.
     * @param gripperPin GPIO pin attached to gripper servo.
     * @param elevationPin GPIO pin attached to elevation servo.
     * @param yawPin GPIO pin attached to yaw servo.
     */
    void init(int gripperPin, int elevationPin, int yawPin);

    /**
     * @brief Filters target angles and updates physical servo PWM microsecond outputs.
     * @param targetRoll Target roll angle in degrees.
     * @param targetPitch Target pitch angle in degrees.
     * @param targetYaw Target yaw angle in degrees.
     */
    void update(float targetRoll, float targetPitch, float targetYaw);
    
    /**
     * @brief Reads last written gripper servo angle.
     * @return Gripper servo angle in degrees.
     */
    int getGripperAngle();

    /**
     * @brief Reads last written elevation servo angle.
     * @return Elevation servo angle in degrees.
     */
    int getElevationAngle();

    /**
     * @brief Reads last written yaw servo angle.
     * @return Yaw servo angle in degrees.
     */
    int getYawAngle();
    
    /**
     * @brief Sets LPF smoothing parameters for Roll, Pitch, Yaw.
     * @param alphaRoll Smoothing factor for roll [0.0 to 1.0].
     * @param alphaPitch Smoothing factor for pitch [0.0 to 1.0].
     * @param alphaYaw Smoothing factor for yaw [0.0 to 1.0].
     */
    void setFilterAlphas(float alphaRoll, float alphaPitch, float alphaYaw);

    /**
     * @brief Sets continuous deadband thresholds.
     * @param deadbandRoll Deadband threshold for roll in degrees.
     * @param deadbandPitch Deadband threshold for pitch in degrees.
     * @param deadbandYaw Deadband threshold for yaw in degrees.
     */
    void setDeadbands(float deadbandRoll, float deadbandPitch, float deadbandYaw);

    /**
     * @brief Sets maximum angular velocity slew-rate limits.
     * @param maxDegPerSecRoll Maximum roll change in deg/s.
     * @param maxDegPerSecPitch Maximum pitch change in deg/s.
     * @param maxDegPerSecYaw Maximum yaw change in deg/s.
     */
    void setSlewRateLimits(float maxDegPerSecRoll, float maxDegPerSecPitch, float maxDegPerSecYaw);

    /**
     * @brief Sets physical IMU input range mapping for roll axis.
     * @param minImuRoll Minimum IMU roll angle mapped to 0° servo.
     * @param maxImuRoll Maximum IMU roll angle mapped to 90° servo.
     */
    void setRollRange(float minImuRoll, float maxImuRoll);

    /**
     * @brief Sets physical IMU input range mapping for pitch axis.
     * @param minImuPitch Minimum IMU pitch angle mapped to 0° servo.
     * @param maxImuPitch Maximum IMU pitch angle mapped to 180° servo.
     */
    void setPitchRange(float minImuPitch, float maxImuPitch);

    /**
     * @brief Sets physical IMU input range mapping for yaw axis.
     * @param minImuYaw Minimum IMU yaw angle mapped to 0° servo.
     * @param maxImuYaw Maximum IMU yaw angle mapped to 180° servo.
     */
    void setYawRange(float minImuYaw, float maxImuYaw);

private:
    Servo _gripperServo;
    Servo _elevationServo;
    Servo _yawServo;

    float _currentRoll;
    float _currentPitch;
    float _currentYaw;

    float _alphaRoll;
    float _alphaPitch;
    float _alphaYaw;
    
    float _deadbandRoll;
    float _deadbandPitch;
    float _deadbandYaw;

    float _maxSlewRoll;   // Max deg/sec
    float _maxSlewPitch;  // Max deg/sec
    float _maxSlewYaw;    // Max deg/sec

    float _minImuRoll;    // Min physical IMU roll mapped to 0 deg Servo (500us)
    float _maxImuRoll;    // Max physical IMU roll mapped to 90 deg Servo (1500us)
    float _minImuPitch;   // Min physical IMU pitch mapped to 0 deg Servo (500us)
    float _maxImuPitch;   // Max physical IMU pitch mapped to 180 deg Servo (2500us)
    float _minImuYaw;     // Min physical IMU yaw mapped to 0 deg Servo (500us)
    float _maxImuYaw;     // Max physical IMU yaw mapped to 180 deg Servo (2500us)
    
    unsigned long _lastUpdateMicros;
};

#endif
