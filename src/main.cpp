#include <Arduino.h>
#include <Wire.h>
#include "BNO085_HAL.h"
#include "AttitudeEstimator.h"
#include "SensorCalibration.h"
#include "config.h"

// --- Pin Definitions ---
#define I2C_SDA 21
#define I2C_SCL 22
#define BNO08X_INT 4 // Hardware Interrupt Pin

// --- Hardware Instances ---
BNO085_HAL imu;
AttitudeEstimator estimator(&imu);

// --- Timing Variables ---
unsigned long lastPrintTime = 0;
unsigned long sampleCount = 0;
unsigned long lastSampleTime = 0;

// --- Performance Evaluation Metrics ---
unsigned long maxUpdateMicros = 0;   // Tracks worst-case execution spike
float avgUpdateMicros = 0.0f;        // Running Exponential Moving Average (EMA)
const float EMA_ALPHA = 0.01f;       // Smoothing weight factor (looks at last ~100 samples)

// Forward declaration of the timing stress test
void runFilterSpeedStressTest();

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n--- BNO085 Hardware Verification Test ---");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeOut(200);
    Wire.setClock(400000); 
    
    if (!imu.init(true, false)) {
        Serial.println("ERROR: Failed to initialize BNO085!");
        while (1) { delay(10); }
    }
    Serial.println("BNO085 Initialized Successfully!");

    estimator.selectFilter(AttitudeFilterSel::ESKF);
    estimator.setUseMagnetometer(true);
    estimator.setMagneticLocation(MagLocation::MUNICH);
    
    // --- Run Isolated Computational Stress Test ---
    runFilterSpeedStressTest();
    // SensorCalibration::runDynamicCalibration(&imu); // Fast Accel/Mag scale and offsets (Rotate sensor)
    // SensorCalibration::runTumbleCalibration(&imu);  // High-Precision Accel/Mag (Static poses via Serial 's')
    // SensorCalibration::runStaticCalibration(&imu);  // For Gyro offset and Noise variances (Keep still)
    // SensorCalibration::runAllanVarianceCollection(&imu, 1800000); // 30min test at 100Hz

    
    lastSampleTime = micros();
}

void loop() {
    // Poll the hardware for new data integration 
    if (imu.update()) {
        
        // Calculate the exact dt for the Kalman Filter
        unsigned long currentMicros = micros();
        float dt = (currentMicros - lastSampleTime) / 1000000.0f;
        lastSampleTime = currentMicros;

        float ax = imu.getAccX();
        float ay = imu.getAccY();
        float az = imu.getAccZ();
        float gx = imu.getGyroX_rads();
        float gy = imu.getGyroY_rads();
        float gz = imu.getGyroZ_rads();
        float mx = imu.getMagX();
        float my = imu.getMagY();
        float mz = imu.getMagZ();

        //  Measure Filter Update Latency 
        unsigned long startCompute = micros();

        // Feed the data into the wrapper 
        estimator.update(dt, false);

        unsigned long endCompute = micros();

        // Calculate metrics
        unsigned long currentUpdateMicros = endCompute - startCompute;
        
        if (currentUpdateMicros > maxUpdateMicros) {
            maxUpdateMicros = currentUpdateMicros; // Capture worst-case latency spike
        }

        if (avgUpdateMicros == 0.0f) {
            avgUpdateMicros = (float)currentUpdateMicros; // Initialize filter
        } else {
            // Apply Exponential Moving Average filter
            avgUpdateMicros = (EMA_ALPHA * currentUpdateMicros) + ((1.0f - EMA_ALPHA) * avgUpdateMicros);
        }

        // Extract the filtered angles
        float roll = estimator.getRoll();
        float pitch = estimator.getPitch();
        float yaw = estimator.getYaw();

        // Print debug data every 50ms 
        if (millis() - lastPrintTime >= 50) {
            lastPrintTime = millis();
            
            // Standard Text Readout for Serial Monitor
            Serial.printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Comp Time: %lu us (Avg: %.2f us, Max: %lu us)\n",
                          roll, pitch, yaw, currentUpdateMicros, avgUpdateMicros, maxUpdateMicros);
            

            Serial.printf(
                ">Roll:%f\n>Pitch:%f\n>Yaw:%f\n"
                ">ax:%f\n>ay:%f\n>az:%f\n"
                ">gx:%f\n>gy:%f\n>gz:%f\n"
                ">mx:%f\n>my:%f\n>mz:%f\n",
                roll, pitch, yaw,
                estimator.getTransformedAccX(), estimator.getTransformedAccY(), estimator.getTransformedAccZ(),
                estimator.getTransformedGyroX(), estimator.getTransformedGyroY(), estimator.getTransformedGyroZ(),
                estimator.getTransformedMagX(), estimator.getTransformedMagY(), estimator.getTransformedMagZ()
            );
        }
    }
}

/**
 * @brief Isolated Stress Test Function
 * Evaluates the execution time of the filter update function repeatedly 
 * without waiting for I2C data streams to establish a pure mathematical runtime profile.
 */
void runFilterSpeedStressTest() {
    Serial.println("\n=====================================================");
    Serial.println("  RUNNING FILTER COMPUTATIONAL SPEED STRESS TEST     ");
    Serial.println("=====================================================");
    Serial.println("Executing 1,000 baseline updates over static cache...");
    delay(500); // Allow serial buffer to settle

    const int iterations = 1000;
    const float simulated_dt = 0.01f; // Fixed 100Hz step simulation
    
    unsigned long startTest = micros();
    for (int i = 0; i < iterations; i++) {
        // We do not call imu.update() here. This keeps sensor readings static 
        // inside the HAL layer cache, isolating purely the filter's backend math processing.
        estimator.update(simulated_dt, false);
    }
    unsigned long endTest = micros();
    
    unsigned long totalTimeMicros = endTest - startTest;
    float averageCallMicros = (float)totalTimeMicros / (float)iterations;

    Serial.printf(">> Test Completed for %d Iterations.\n", iterations);
    Serial.printf(">> Total Computation Duration : %lu us\n", totalTimeMicros);
    Serial.printf(">> Isolated Pure Filter Cost  : %.3f us per call\n", averageCallMicros);
    Serial.println("=====================================================\n");
    
}

