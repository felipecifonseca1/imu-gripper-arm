#include "BNO085_HAL.h"

BNO085_HAL::BNO085_HAL(uint8_t i2cAddr) : _i2cAddr(i2cAddr) {
}

bool BNO085_HAL::init(bool verbose, bool autoCalibrate) {
    if (verbose) {
        Serial.println("Initializing BNO085...");
    }

    if (!_bno.begin_I2C(_i2cAddr, &Wire, 0)) {
        if (verbose) Serial.println("ERROR: Failed to find BNO085!");
        return false;
    }

    // Enable reports at 100Hz (10,000 microseconds)
    if (!_bno.enableReport(SH2_ACCELEROMETER, 10000)) {
        if (verbose) Serial.println("Could not enable Accelerometer");
    }
    if (!_bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000)) {
        if (verbose) Serial.println("Could not enable Gyroscope (Uncalibrated)");
    }
    if (!_bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000)) {
        if (verbose) Serial.println("Could not enable Magnetometer");
    }

    if (verbose) {
        Serial.println("BNO085 Initialized Successfully!");
    }

    return true;
}

bool BNO085_HAL::update(float dt) {
    if (_hilMode) return true;

    sh2_SensorValue_t sensorValue;
    bool newData = false;

    // Drain the BNO085 event queue to get the freshest data for all enabled sensors
    while (_bno.getSensorEvent(&sensorValue)) {
        newData = true;
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                // Convert m/s^2 to g
                _accX = sensorValue.un.accelerometer.x / GRAVITY_EARTH;
                _accY = sensorValue.un.accelerometer.y / GRAVITY_EARTH;
                _accZ = sensorValue.un.accelerometer.z / GRAVITY_EARTH;
                break;
            case SH2_GYROSCOPE_UNCALIBRATED:
                // Natively in rad/s (raw/uncalibrated)
                _gyroX = sensorValue.un.gyroscopeUncal.x;
                _gyroY = sensorValue.un.gyroscopeUncal.y;
                _gyroZ = sensorValue.un.gyroscopeUncal.z;
                break;
            case SH2_MAGNETIC_FIELD_CALIBRATED:
                // Natively in uT
                _magX = sensorValue.un.magneticField.x;
                _magY = sensorValue.un.magneticField.y;
                _magZ = sensorValue.un.magneticField.z;
                break;
        }
    }

    return newData;
}

void BNO085_HAL::injectData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    _hilMode = true;
    _accX = ax; 
    _accY = ay; 
    _accZ = az;
    _gyroX = gx; 
    _gyroY = gy; 
    _gyroZ = gz;
    _magX = mx; 
    _magY = my; 
    _magZ = mz;
}
