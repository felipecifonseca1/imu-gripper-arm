#include <Arduino.h>
#include <Wire.h>
#include "BNO085_HAL.h"
#include "AttitudeEstimator.h"
#include "SensorCalibration.h"
#include "config.h"
#include "ServoController.h"

// --- Pin Definitions ---
#define I2C_SDA 21
#define I2C_SCL 22
#define BNO08X_INT 4 // Hardware Interrupt Pin
#define GRIPPER_PIN 5 
#define ELEVATION_PIN 18
#define YAW_PIN 19

// --- Hardware Instances ---
BNO085_HAL imu;
AttitudeEstimator estimator(&imu);
ServoController servoController;

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

    servoController.init(GRIPPER_PIN, ELEVATION_PIN, YAW_PIN);
    Serial.println("\n--- Base Receiver & Servo Setup Complete ---");
    Serial.println("Send 'roll,pitch' via Serial or Bluetooth for manual override, otherwise IMU takes control.");

    lastSampleTime = micros();
}

void loop() {
    // --- 1. Read from USB Serial Monitor (Manual Override) ---
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            int commaIndex = input.indexOf(',');
            if (commaIndex > 0) {
                float targetRoll = input.substring(0, commaIndex).toFloat();
                float targetPitch = input.substring(commaIndex + 1).toFloat();
                
                Serial.printf("Test Input -> Target Roll: %.2f | Target Pitch: %.2f\n", targetRoll, targetPitch);
                // When sending manual commands, loop to let lowpass filter reach target
                for(int i=0; i<50; i++) {
                    servoController.update(targetRoll, targetPitch, 0.0f);
                    delay(20);
                }
            }
        }
    }


    // --- 2. Poll hardware and update IMU & Servos ---
    if (imu.update()) {
        unsigned long currentMicros = micros();
        float dt = (currentMicros - lastSampleTime) / 1000000.0f;
        lastSampleTime = currentMicros;

        unsigned long startCompute = micros();
        estimator.update(dt, false);
        unsigned long endCompute = micros();

        unsigned long currentUpdateMicros = endCompute - startCompute;
        
        if (currentUpdateMicros > maxUpdateMicros) {
            maxUpdateMicros = currentUpdateMicros;
        }

        if (avgUpdateMicros == 0.0f) {
            avgUpdateMicros = (float)currentUpdateMicros;
        } else {
            avgUpdateMicros = (EMA_ALPHA * currentUpdateMicros) + ((1.0f - EMA_ALPHA) * avgUpdateMicros);
        }

        float roll = estimator.getRoll();
        float pitch = estimator.getPitch();
        float yaw = estimator.getYaw();

        // Update servos with IMU data
        servoController.update(roll, pitch, yaw);

        // Print debug data every 50ms 
        if (millis() - lastPrintTime >= 50) {
            lastPrintTime = millis();
            
            Serial.printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Comp Time: %lu us (Avg: %.2f us, Max: %lu us)\n",
                          roll, pitch, yaw, currentUpdateMicros, avgUpdateMicros, maxUpdateMicros);
            
            Serial.printf(
                ">Roll:%f\n>Pitch:%f\n>Yaw:%f\n"
                ">ax:%f\n>ay:%f\n>az:%f\n"
                ">gx:%f\n>gy:%f\n>gz:%f\n"
                ">mx:%f\n>my:%f\n>mz:%f\n"
                ">Servo1_Virtual:%d\n>Servo2_Virtual:%d\n>Servo3_Virtual:%d\n",
                roll, pitch, yaw,
                estimator.getTransformedAccX(), estimator.getTransformedAccY(), estimator.getTransformedAccZ(),
                estimator.getTransformedGyroX(), estimator.getTransformedGyroY(), estimator.getTransformedGyroZ(),
                estimator.getTransformedMagX(), estimator.getTransformedMagY(), estimator.getTransformedMagZ(),
                servoController.getGripperAngle() - 90, servoController.getElevationAngle() - 90, servoController.getYawAngle() - 90
            );
        }
    }
}

/**
 * @brief Isolated Stress Test Function
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
