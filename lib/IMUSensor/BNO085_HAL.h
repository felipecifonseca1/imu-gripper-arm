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
    /**
     * @brief Constructs BNO085 HAL instance.
     * @param i2cAddr I2C bus address of BNO085.
     */
    BNO085_HAL(uint8_t i2cAddr = BNO08x_I2CADDR_DEFAULT);

    /**
     * @brief Initializes BNO085 sensor over I2C and enables report channels.
     * @param verbose Flag to enable debug log outputs.
     * @param autoCalibrate Flag to trigger automatic calibration sequence.
     * @return True if initialized successfully, false otherwise.
     */
    bool init(bool verbose = true, bool autoCalibrate = false) override;

    /**
     * @brief Polls BNO085 sensor event queue.
     * @param dt Integration time step in seconds.
     * @return True if new data was received, false otherwise.
     */
    bool update(float dt = 0.02f) override;

    /**
     * @brief Injects simulated sensor data for HIL mode.
     * @param ax Accelerometer X in g.
     * @param ay Accelerometer Y in g.
     * @param az Accelerometer Z in g.
     * @param gx Gyroscope X in rad/s.
     * @param gy Gyroscope Y in rad/s.
     * @param gz Gyroscope Z in rad/s.
     * @param mx Magnetometer X in uT.
     * @param my Magnetometer Y in uT.
     * @param mz Magnetometer Z in uT.
     */
    void injectData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) override;

    /**
     * @brief Triggers accelerometer calibration routine.
     */
    void calibrateAccel() override {}

    /**
     * @brief Triggers gyroscope calibration routine.
     */
    void calibrateGyro() override {}

    /**
     * @brief Triggers magnetometer calibration routine.
     */
    void calibrateMag() override {}

    /**
     * @brief Gets current accelerometer X reading.
     * @return Accelerometer X value in g.
     */
    float getAccX() const override { return _accX; }

    /**
     * @brief Gets current accelerometer Y reading.
     * @return Accelerometer Y value in g.
     */
    float getAccY() const override { return _accY; }

    /**
     * @brief Gets current accelerometer Z reading.
     * @return Accelerometer Z value in g.
     */
    float getAccZ() const override { return _accZ; }

    /**
     * @brief Gets current gyroscope X reading.
     * @return Gyroscope X value in rad/s.
     */
    float getGyroX_rads() const override { return _gyroX; }

    /**
     * @brief Gets current gyroscope Y reading.
     * @return Gyroscope Y value in rad/s.
     */
    float getGyroY_rads() const override { return _gyroY; }

    /**
     * @brief Gets current gyroscope Z reading.
     * @return Gyroscope Z value in rad/s.
     */
    float getGyroZ_rads() const override { return _gyroZ; }

    /**
     * @brief Gets current magnetometer X reading.
     * @return Magnetometer X value in uT.
     */
    float getMagX() const override { return _magX; }

    /**
     * @brief Gets current magnetometer Y reading.
     * @return Magnetometer Y value in uT.
     */
    float getMagY() const override { return _magY; }

    /**
     * @brief Gets current magnetometer Z reading.
     * @return Magnetometer Z value in uT.
     */
    float getMagZ() const override { return _magZ; }

    /**
     * @brief Gets temperature reading.
     * @return Temperature in degrees Celsius.
     */
    float getTemperature() const override { return 0.0f; }

    /**
     * @brief Gets accelerometer X bias.
     * @return Accelerometer X bias in g.
     */
    float getAccelBiasX() const override { return 0.0f; }

    /**
     * @brief Gets accelerometer Y bias.
     * @return Accelerometer Y bias in g.
     */
    float getAccelBiasY() const override { return 0.0f; }

    /**
     * @brief Gets accelerometer Z bias.
     * @return Accelerometer Z bias in g.
     */
    float getAccelBiasZ() const override { return 0.0f; }

    /**
     * @brief Gets gyroscope X bias.
     * @return Gyroscope X bias in rad/s.
     */
    float getGyroBiasX() const override { return 0.0f; }

    /**
     * @brief Gets gyroscope Y bias.
     * @return Gyroscope Y bias in rad/s.
     */
    float getGyroBiasY() const override { return 0.0f; }

    /**
     * @brief Gets gyroscope Z bias.
     * @return Gyroscope Z bias in rad/s.
     */
    float getGyroBiasZ() const override { return 0.0f; }

    /**
     * @brief Gets device name string.
     * @return Name string of the sensor device.
     */
    String getDeviceName() const override { return "Adafruit_BNO085"; }
    
    /**
     * @brief Accesses underlying Adafruit BNO08x object reference.
     * @return Reference to raw Adafruit_BNO08x device driver object.
     */
    Adafruit_BNO08x& getRawDevice() { return _bno; }
};

#endif // BNO085_HAL_H
