#include "SensorCalibration.h"
#include <math.h>

using namespace Eigen;

bool SensorCalibration::calibrateEllipsoid(const std::vector<Vector3f>& data, Matrix3f& G_out, Vector3f& w_out) {
    if (data.size() < 6) return false;

    int N = data.size();
    MatrixXf D(N, 6);
    VectorXf O = VectorXf::Ones(N);

    for (int i = 0; i < N; ++i) {
        float x = data[i].x();
        float y = data[i].y();
        float z = data[i].z();
        D(i, 0) = x * x;
        D(i, 1) = y * y;
        D(i, 2) = z * z;
        D(i, 3) = x;
        D(i, 4) = y;
        D(i, 5) = z;
    }

    // Solve the overdetermined linear system D * p = O
    // Using SVD for maximum numerical stability
    VectorXf p = D.bdcSvd(ComputeThinU | ComputeThinV).solve(O);

    float A = p(0);
    float B = p(1);
    float C = p(2);
    float D_lin = p(3);
    float E = p(4);
    float F = p(5);

    // If A, B, or C are exactly 0, data is too poor to invert
    if (abs(A) < 1e-6f || abs(B) < 1e-6f || abs(C) < 1e-6f) {
        return false;
    }

    Vector3f v(D_lin / 2.0f, E / 2.0f, F / 2.0f);

    // Calculate center offset w = -A_fit^-1 * v
    // Since A_fit is diagonal [A, B, C], the inverse is just 1/val
    w_out(0) = -v(0) / A;
    w_out(1) = -v(1) / B;
    w_out(2) = -v(2) / C;

    // Calculate algebraic to geometric scale factor k
    float k = 1.0f + (v(0) * v(0) / A + v(1) * v(1) / B + v(2) * v(2) / C);
    
    // Safety check for k
    if (abs(k) < 1e-6f) return false;

    // Calculate Gain matrix G
    G_out.setZero();
    G_out(0, 0) = sqrt(abs(A / k));
    G_out(1, 1) = sqrt(abs(B / k));
    G_out(2, 2) = sqrt(abs(C / k));

    return true;
}

bool SensorCalibration::calibrateStatic(const std::vector<Vector3f>& data, Vector3f& w_out) {
    if (data.empty()) return false;

    w_out.setZero();
    for (const auto& sample : data) {
        w_out += sample;
    }
    w_out /= (float)data.size();

    return true;
}

bool SensorCalibration::calibrateNoise(const std::vector<Vector3f>& data, float& variance_out) {
    if (data.size() < 2) return false;

    // Step 1: Calculate Mean
    Vector3f mean = Vector3f::Zero();
    for (const auto& sample : data) {
        mean += sample;
    }
    mean /= (float)data.size();

    // Step 2: Calculate Variance per axis
    Vector3f var_sum = Vector3f::Zero();
    for (const auto& sample : data) {
        Vector3f diff = sample - mean;
        var_sum(0) += diff(0) * diff(0);
        var_sum(1) += diff(1) * diff(1);
        var_sum(2) += diff(2) * diff(2);
    }
    
    // Sample variance (N-1)
    Vector3f variance = var_sum / (float)(data.size() - 1);

    // Average the variance across X, Y, Z to get a scalar noise value
    variance_out = (variance(0) + variance(1) + variance(2)) / 3.0f;

    return true;
}

// --- High-Level Workflow Functions ---

/**
 * @brief Collects raw gyro data over a specified duration for Allan Variance analysis.
 * @param imu Pointer to IMUSensor instance.
 * @param durationMs Collection duration in milliseconds.
 */
void SensorCalibration::runAllanVarianceCollection(IMUSensor* imu, unsigned long durationMs) {
    Serial.println("\n--- IMU ALLAN VARIANCE COLLECTION ---");
    Serial.printf("Running for %lu ms at 100Hz.\n", durationMs);
    Serial.println("Leave sensor completely stationary.");
    Serial.println("START_ALLAN_DATA");
    
    unsigned long startTime = millis();
    unsigned long lastSampleTime = startTime;
    
    while (millis() - startTime < durationMs) {
        if (imu->update()) {
            unsigned long currentMillis = millis();
            // Target 100Hz = 10ms
            if (currentMillis - lastSampleTime >= 10) {
                // Print timestamp (ms) and raw gyro rad/s
                Serial.printf("%lu,%f,%f,%f\n", 
                    currentMillis - startTime,
                    imu->getGyroX_rads(),
                    imu->getGyroY_rads(),
                    imu->getGyroZ_rads());
                lastSampleTime = currentMillis;
            }
        }
        delay(1);
    }
    
    Serial.println("END_ALLAN_DATA");
    Serial.println("Allan Variance Collection complete.");
    while(1) { delay(100); }
}

/**
 * @brief Runs dynamic ellipsoid calibration routine requiring sensor rotation.
 * @param imu Pointer to IMUSensor instance.
 */
void SensorCalibration::runDynamicCalibration(IMUSensor* imu) {
    Serial.println("\n--- IMU DYNAMIC CALIBRATION ---");
    Serial.println("Rotate sensor in all directions. Collecting 1000 averaged samples (takes ~20s)...");
    
    std::vector<Eigen::Vector3f> accelData;
    std::vector<Eigen::Vector3f> magData;
    
    accelData.reserve(1000);
    magData.reserve(1000);
    
    int samples = 0;
    int accumCount = 0;
    Eigen::Vector3f accelAccum = Eigen::Vector3f::Zero();
    Eigen::Vector3f magAccum = Eigen::Vector3f::Zero();
    unsigned long lastSampleTime = millis();
    
    while (samples < 1000) {
        if (imu->update()) {
            accelAccum += Eigen::Vector3f(imu->getAccX(), imu->getAccY(), imu->getAccZ());
            magAccum += Eigen::Vector3f(imu->getMagX(), imu->getMagY(), imu->getMagZ());
            accumCount++;
        }
        
        // Save average every 20ms (50Hz) -> 1000 samples = 20 seconds
        if (millis() - lastSampleTime >= 20) {
            if (accumCount > 0) {
                accelData.push_back(accelAccum / (float)accumCount);
                magData.push_back(magAccum / (float)accumCount);
                samples++;
                if (samples % 100 == 0) Serial.printf("Progress: %d/1000\n", samples);
            }
            accelAccum.setZero();
            magAccum.setZero();
            accumCount = 0;
            lastSampleTime = millis();
        }
        delay(1);
    }
    
    Eigen::Matrix3f G_acc, G_mag;
    Eigen::Vector3f w_acc, w_mag;
    
    Serial.println("\nCalculating Accelerometer Calibration (Ellipsoid Fit)...");
    if (calibrateEllipsoid(accelData, G_acc, w_acc)) {
        Serial.printf("constexpr float ACCEL_OFFSET[3] = {%ff, %ff, %ff};\n", w_acc(0), w_acc(1), w_acc(2));
        Serial.printf("constexpr float ACCEL_GAIN[9] = {%ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff};\n", 
                      G_acc(0,0), G_acc(0,1), G_acc(0,2), G_acc(1,0), G_acc(1,1), G_acc(1,2), G_acc(2,0), G_acc(2,1), G_acc(2,2));
    } else { Serial.println("Accel calibration failed."); }
    
    Serial.println("\nCalculating Magnetometer Calibration (Ellipsoid Fit)...");
    if (calibrateEllipsoid(magData, G_mag, w_mag)) {
        Serial.printf("constexpr float MAG_OFFSET[3] = {%ff, %ff, %ff};\n", w_mag(0), w_mag(1), w_mag(2));
        Serial.printf("constexpr float MAG_GAIN[9] = {%ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff};\n", 
                      G_mag(0,0), G_mag(0,1), G_mag(0,2), G_mag(1,0), G_mag(1,1), G_mag(1,2), G_mag(2,0), G_mag(2,1), G_mag(2,2));
    } else { Serial.println("Mag calibration failed."); }
    
    Serial.println("\nDynamic Calibration complete. Copy the arrays above into include/config.h");
    while (1) { delay(100); } // Halt
}

/**
 * @brief Runs interactive static tumble calibration using multi-position sampling.
 * @param imu Pointer to IMUSensor instance.
 */
void SensorCalibration::runTumbleCalibration(IMUSensor* imu) {
    Serial.println("\n--- IMU HIGH-PRECISION TUMBLE CALIBRATION ---");
    Serial.println("1. Place sensor in a static orientation.");
    Serial.println("2. Send 's' via Serial Monitor to capture 100 points.");
    Serial.println("3. Repeat for at least 6-10 different orientations (up, down, left, right, tilted).");
    Serial.println("4. Send 'd' when done to compute matrices.");
    
    std::vector<Eigen::Vector3f> accelData;
    std::vector<Eigen::Vector3f> magData;
    
    int poseCount = 0;
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 's') {
                Serial.printf("Capturing Pose %d... Keep still!\n", poseCount + 1);
                delay(1000); // Wait 1s for hands to leave the sensor
                int captured = 0;
                while (captured < 100) {
                    if (imu->update()) {
                        accelData.push_back(Eigen::Vector3f(imu->getAccX(), imu->getAccY(), imu->getAccZ()));
                        magData.push_back(Eigen::Vector3f(imu->getMagX(), imu->getMagY(), imu->getMagZ()));
                        captured++;
                    }
                    delay(5);
                }
                poseCount++;
                Serial.printf("Pose %d Captured. Total points: %zu. Send 's' for next, or 'd' to finish.\n", poseCount, accelData.size());
            } else if (c == 'd') {
                if (poseCount < 6) {
                    Serial.println("Error: Need at least 6 poses for a 3D ellipsoid fit!");
                } else {
                    break;
                }
            }
        }
        imu->update(); // Keep FIFO clear
        delay(10);
    }
    
    Eigen::Matrix3f G_acc, G_mag;
    Eigen::Vector3f w_acc, w_mag;
    
    Serial.println("\nCalculating Accelerometer Calibration (Ellipsoid Fit)...");
    if (calibrateEllipsoid(accelData, G_acc, w_acc)) {
        Serial.printf("constexpr float ACCEL_OFFSET[3] = {%ff, %ff, %ff};\n", w_acc(0), w_acc(1), w_acc(2));
        Serial.printf("constexpr float ACCEL_GAIN[9] = {%ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff};\n", 
                      G_acc(0,0), G_acc(0,1), G_acc(0,2), G_acc(1,0), G_acc(1,1), G_acc(1,2), G_acc(2,0), G_acc(2,1), G_acc(2,2));
    } else { Serial.println("Accel calibration failed."); }
    
    Serial.println("\nCalculating Magnetometer Calibration (Ellipsoid Fit)...");
    if (calibrateEllipsoid(magData, G_mag, w_mag)) {
        Serial.printf("constexpr float MAG_OFFSET[3] = {%ff, %ff, %ff};\n", w_mag(0), w_mag(1), w_mag(2));
        Serial.printf("constexpr float MAG_GAIN[9] = {%ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff};\n", 
                      G_mag(0,0), G_mag(0,1), G_mag(0,2), G_mag(1,0), G_mag(1,1), G_mag(1,2), G_mag(2,0), G_mag(2,1), G_mag(2,2));
    } else { Serial.println("Mag calibration failed."); }
    
    Serial.println("\nTumble Calibration complete. Copy the arrays above into include/config.h");
    while (1) { delay(100); } // Halt
}

/**
 * @brief Runs static zero-rate offset calibration and estimates noise variances.
 * @param imu Pointer to IMUSensor instance.
 */
void SensorCalibration::runStaticCalibration(IMUSensor* imu) {
    Serial.println("\n--- IMU STATIC CALIBRATION ---");
    Serial.println("Leave the sensor completely stationary on a flat surface. Collecting 1000 samples...");
    
    std::vector<Eigen::Vector3f> accelData;
    std::vector<Eigen::Vector3f> gyroData;
    std::vector<Eigen::Vector3f> magData;
    
    accelData.reserve(1000);
    gyroData.reserve(1000);
    magData.reserve(1000);
    
    int samples = 0;
    while (samples < 1000) {
        if (imu->update()) {
            accelData.push_back(Eigen::Vector3f(imu->getAccX(), imu->getAccY(), imu->getAccZ()));
            gyroData.push_back(Eigen::Vector3f(imu->getGyroX_rads(), imu->getGyroY_rads(), imu->getGyroZ_rads()));
            magData.push_back(Eigen::Vector3f(imu->getMagX(), imu->getMagY(), imu->getMagZ()));
            samples++;
            if (samples % 100 == 0) Serial.printf("Progress: %d/1000\n", samples);
        }
        delay(1);
    }
    
    Eigen::Vector3f w_gyro;
    Serial.println("\nCalculating Gyroscope Calibration (Static ZRO)...");
    if (calibrateStatic(gyroData, w_gyro)) {
        Serial.printf("constexpr float GYRO_OFFSET[3] = {%ff, %ff, %ff};\n", w_gyro(0), w_gyro(1), w_gyro(2));
    }
    
    Serial.println("\n--- NOISE ESTIMATION ---");
    float accel_var, gyro_var, mag_var;
    if (calibrateNoise(accelData, accel_var) &&
        calibrateNoise(gyroData, gyro_var) &&
        calibrateNoise(magData, mag_var)) {
        Serial.printf("const float AccelerometerNoise = %.8ef;\n", accel_var);
        Serial.printf("const float GyroscopeNoise = %.8ef;\n", gyro_var);
        Serial.printf("const float MagnetometerNoise = %.8ef;\n", mag_var);
    }
    
    Serial.println("\nStatic Calibration complete. Copy the arrays/noise above into your config.");
    while (1) { delay(100); } // Halt
}
