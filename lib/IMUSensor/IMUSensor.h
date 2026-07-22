#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>

/**
 * @class IMUSensor
 * @brief Abstract base class defining the hardware interface for an Inertial Measurement Unit.
 * @details This contract promises raw, unfiltered data from the Accelerometer, Gyroscope, and Magnetometer.
 */
class IMUSensor {
public:
    /**
     * @brief Virtual destructor for IMUSensor interface.
     */
    virtual ~IMUSensor() = default;

    // --- Hardware Control ---
    /**
     * @brief Initializes the IMU hardware and configures settings.
     * @param verbose Flag to enable serial debug output.
     * @param autoCalibrate Flag to trigger automatic calibration sequence.
     * @return True if initialization was successful, false otherwise.
     */
    virtual bool init(bool verbose = true, bool autoCalibrate = false) = 0;

    /**
     * @brief Polls the hardware for the latest data.
     * @param dt Integration time step in seconds.
     * @return True if new data was successfully read, false otherwise.
     */
    virtual bool update(float dt = 0.02f) = 0;

    /**
     * @brief Injects external sensor data for Hardware-In-the-Loop (HIL) simulations.
     * @param ax Accelerometer X reading in g.
     * @param ay Accelerometer Y reading in g.
     * @param az Accelerometer Z reading in g.
     * @param gx Gyroscope X angular rate in rad/s.
     * @param gy Gyroscope Y angular rate in rad/s.
     * @param gz Gyroscope Z angular rate in rad/s.
     * @param mx Magnetometer X flux in uT.
     * @param my Magnetometer Y flux in uT.
     * @param mz Magnetometer Z flux in uT.
     */
    virtual void injectData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) = 0;
    
    // --- Calibration Triggers ---
    /**
     * @brief Triggers accelerometer calibration routine.
     */
    virtual void calibrateAccel() = 0;

    /**
     * @brief Triggers gyroscope calibration routine.
     */
    virtual void calibrateGyro() = 0;

    /**
     * @brief Triggers magnetometer calibration routine.
     */
    virtual void calibrateMag() = 0;

    // --- Raw Accelerometer Data (g) ---
    /**
     * @brief Gets raw accelerometer X measurement.
     * @return Accelerometer X reading in g.
     */
    virtual float getAccX() const = 0;

    /**
     * @brief Gets raw accelerometer Y measurement.
     * @return Accelerometer Y reading in g.
     */
    virtual float getAccY() const = 0;

    /**
     * @brief Gets raw accelerometer Z measurement.
     * @return Accelerometer Z reading in g.
     */
    virtual float getAccZ() const = 0;

    // --- Raw Gyroscope Data (rad/s) ---
    /**
     * @brief Gets raw gyroscope X measurement.
     * @return Gyroscope X angular rate in rad/s.
     */
    virtual float getGyroX_rads() const = 0;

    /**
     * @brief Gets raw gyroscope Y measurement.
     * @return Gyroscope Y angular rate in rad/s.
     */
    virtual float getGyroY_rads() const = 0;

    /**
     * @brief Gets raw gyroscope Z measurement.
     * @return Gyroscope Z angular rate in rad/s.
     */
    virtual float getGyroZ_rads() const = 0;

    // --- Raw Magnetometer Data (uT) ---
    /**
     * @brief Gets raw magnetometer X measurement.
     * @return Magnetometer X flux in uT.
     */
    virtual float getMagX() const = 0;

    /**
     * @brief Gets raw magnetometer Y measurement.
     * @return Magnetometer Y flux in uT.
     */
    virtual float getMagY() const = 0;

    /**
     * @brief Gets raw magnetometer Z measurement.
     * @return Magnetometer Z flux in uT.
     */
    virtual float getMagZ() const = 0;
    
    // --- Temperature (°C) ---
    /**
     * @brief Gets sensor temperature reading.
     * @return Temperature reading in degrees Celsius.
     */
    virtual float getTemperature() const = 0;
    
    // --- Calibration Getters/Setters ---
    /**
     * @brief Gets estimated accelerometer X bias.
     * @return Accelerometer X bias in g.
     */
    virtual float getAccelBiasX() const = 0;

    /**
     * @brief Gets estimated accelerometer Y bias.
     * @return Accelerometer Y bias in g.
     */
    virtual float getAccelBiasY() const = 0;

    /**
     * @brief Gets estimated accelerometer Z bias.
     * @return Accelerometer Z bias in g.
     */
    virtual float getAccelBiasZ() const = 0;
    
    /**
     * @brief Gets estimated gyroscope X bias.
     * @return Gyroscope X bias in rad/s.
     */
    virtual float getGyroBiasX() const = 0;

    /**
     * @brief Gets estimated gyroscope Y bias.
     * @return Gyroscope Y bias in rad/s.
     */
    virtual float getGyroBiasY() const = 0;

    /**
     * @brief Gets estimated gyroscope Z bias.
     * @return Gyroscope Z bias in rad/s.
     */
    virtual float getGyroBiasZ() const = 0;

    // --- Device Information ---
    /**
     * @brief Gets descriptive device string.
     * @return Name of the hardware sensor device.
     */
    virtual String getDeviceName() const = 0;
};

#endif // IMU_SENSOR_H
