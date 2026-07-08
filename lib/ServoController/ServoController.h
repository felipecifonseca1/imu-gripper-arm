#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoController {
public:
    ServoController();
    void init(int gripperPin, int elevationPin);
    void update(float targetRoll, float targetPitch);
    
    void setFilterAlpha(float alpha);
    void setDeadband(float deadband);

private:
    Servo _gripperServo;
    Servo _elevationServo;

    float _currentRoll;
    float _currentPitch;

    float _alpha;
    float _deadband;
};

#endif
