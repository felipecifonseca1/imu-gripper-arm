#ifndef BNO085_HAL_H
#define BNO085_HAL_H

#include "IMUSensor.h"
#include <Adafruit_BNO08x.h>
#include <Wire.h>

class BNO085_HAL : public IMUSensor {
private:
    Adafruit_BNO08x _bno;
    uint8_t _i2cAddr;
    
    // Internal state for the latest sensor readings
    float _accX = 0, _accY = 0, _accZ = 0;
    float _gyroX = 0, _gyroY = 0, _gyroZ = 0;
    float _magX = 0, _magY = 0, _magZ = 0;
    
    bool _hilMode = false;
    
    // BNO085 returns acceleration in m/s^2. Convert to g to match IMUSensor contract.
    static constexpr float GRAVITY_EARTH = 9.80665f;

public:
    BNO085_HAL(uint8_t i2cAddr = BNO08x_I2CADDR_DEFAULT);

    bool init(bool verbose = true, bool autoCalibrate = false) override;
    bool update(float dt = 0.02f) override;
    void injectData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) override;

    // BNO085 performs auto-calibration internally; dummy methods for interface compliance
    void calibrateAccel() override {}
    void calibrateGyro() override {}
    void calibrateMag() override {}

    float getAccX() const override { return _accX; }
    float getAccY() const override { return _accY; }
    float getAccZ() const override { return _accZ; }

    float getGyroX_rads() const override { return _gyroX; } // BNO natively provides rad/s
    float getGyroY_rads() const override { return _gyroY; }
    float getGyroZ_rads() const override { return _gyroZ; }

    float getMagX() const override { return _magX; }
    float getMagY() const override { return _magY; }
    float getMagZ() const override { return _magZ; }

    float getTemperature() const override { return 0.0f; } // Not natively provided by SH2 standard reports

    float getAccelBiasX() const override { return 0.0f; }
    float getAccelBiasY() const override { return 0.0f; }
    float getAccelBiasZ() const override { return 0.0f; }

    float getGyroBiasX() const override { return 0.0f; }
    float getGyroBiasY() const override { return 0.0f; }
    float getGyroBiasZ() const override { return 0.0f; }

    String getDeviceName() const override { return "Adafruit_BNO085"; }
    
    // Pass-through
    Adafruit_BNO08x& getRawDevice() { return _bno; }
};

#endif // BNO085_HAL_H
