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
    
    void setFilterAlpha(float alpha);
    void setDeadband(float deadband);

private:
    Servo _gripperServo;
    Servo _elevationServo;
    Servo _yawServo;

    float _currentRoll;
    float _currentPitch;
    float _currentYaw;

    float _alpha;
    float _deadband;
};

#endif
