#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include <ArduinoEigen.h>
#include <vector>
#include <Arduino.h>
#include "../IMUSensor/IMUSensor.h"

class SensorCalibration {
public:
    /**
     * @brief Computes ellipsoid fit calibration for Accelerometer or Magnetometer.
     * Uses JacobiSVD to solve D^T p = 1.
     * 
     * @param data Cloud of 3D data points collected by rotating the sensor.
     * @param G_out (Output) 3x3 Diagonal Gain matrix.
     * @param w_out (Output) 3x1 Offset vector.
     * @return true if successful, false if insufficient data points.
     */
    static bool calibrateEllipsoid(const std::vector<Eigen::Vector3f>& data, Eigen::Matrix3f& G_out, Eigen::Vector3f& w_out);

    /**
     * @brief Computes static calibration for Gyroscope (Zero-Rate Offset).
     * Calculates the statistical mean of the stationary samples.
     * 
     * @param data Cloud of 3D data points collected while stationary.
     * @param w_out (Output) 3x1 Offset vector (ZRO).
     * @return true if successful, false if empty.
     */
    static bool calibrateStatic(const std::vector<Eigen::Vector3f>& data, Eigen::Vector3f& w_out);

    /**
     * @brief Estimates sensor noise variance (used for Kalman Filter tuning).
     * Computes the variance of the data cloud and averages across X, Y, Z.
     * 
     * @param data Cloud of 3D data points collected while completely stationary.
     * @param variance_out (Output) Estimated scalar variance.
     * @return true if successful, false if empty.
     */
    static bool calibrateNoise(const std::vector<Eigen::Vector3f>& data, float& variance_out);

    // --- High-Level Workflow Functions ---
    /**
     * @brief Collects raw gyro data over a duration for Allan Variance noise analysis.
     * @param imu Pointer to IMUSensor instance.
     * @param durationMs Duration of data collection in milliseconds.
     */
    static void runAllanVarianceCollection(IMUSensor* imu, unsigned long durationMs);

    /**
     * @brief Performs dynamic sphere/ellipsoid fit calibration while sensor is rotated.
     * @param imu Pointer to IMUSensor instance.
     */
    static void runDynamicCalibration(IMUSensor* imu);

    /**
     * @brief Performs interactive static tumble calibration using multi-position sampling.
     * @param imu Pointer to IMUSensor instance.
     */
    static void runTumbleCalibration(IMUSensor* imu);

    /**
     * @brief Performs static zero-rate offset calibration and estimates noise variances.
     * @param imu Pointer to IMUSensor instance.
     */
    static void runStaticCalibration(IMUSensor* imu);
};

#endif // SENSOR_CALIBRATION_H
