#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoController {
public:
    ServoController();
    void init(int gripperPin, int elevationPin, int yawPin);
    void update(float targetRoll, float targetPitch, float targetYaw);
    
    int getGripperAngle();
    int getElevationAngle();
    int getYawAngle();
    
    void setFilterAlphas(float alphaRoll, float alphaPitch, float alphaYaw);
    void setDeadbands(float deadbandRoll, float deadbandPitch, float deadbandYaw);

private:
    Servo _gripperServo;
    Servo _elevationServo;
    Servo _yawServo;

    float _currentRoll;
    float _currentPitch;
    float _currentYaw;

    float _lockedTargetRoll;
    float _lockedTargetPitch;
    float _lockedTargetYaw;

    float _alphaRoll;
    float _alphaPitch;
    float _alphaYaw;
    
    float _deadbandRoll;
    float _deadbandPitch;
    float _deadbandYaw;
};

#endif
