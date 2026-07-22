#include <Arduino.h>
#include <Wire.h>
#include "BNO085_HAL.h"
#include "AttitudeEstimator.h"
#include "SensorCalibration.h"
#include "config.h"
#include "ServoController.h"

// --- Pin Definitions ---
#define I2C_SDA 25
#define I2C_SCL 26
#define BNO08X_INT 34 // Hardware Interrupt Pin
#define GRIPPER_PIN 32 
#define ELEVATION_PIN 33
#define YAW_PIN 27

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

// --- Gripper Toggle State ---
bool gripperOverride = false;        // true = spacebar controls gripper, false = IMU controls
bool gripperOpen = true;             // toggle state: true = open (90°), false = closed (0°)

// Forward declaration of the timing stress test
void runFilterSpeedStressTest();

void setup() {
    Serial.begin(115200);
    unsigned long startWait = millis();
    while (!Serial && millis() - startWait < 3000) delay(10); // Warte max 3 Sekunden auf den Serial Monitor
    
    Serial.println("\n--- BNO085 Hardware Verification Test ---");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeOut(200);
    Wire.setClock(100000);     
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
    servoController.setDeadbands(4.0, 2.0, 2.0);      // Degrees of IMU change needed before servo reacts
    servoController.setFilterAlphas(0.35, 0.3, 0.5); // Higher = faster response, lower = smoother
    
    Serial.println("\n--- Setup Complete ---");
    Serial.println("Send 'roll,pitch' for manual override. Press SPACE to toggle gripper open/closed.");

    lastSampleTime = micros();
}

void loop() {
    // --- 1. Read from USB Serial Monitor ---
    if (Serial.available()) {
        String rawInput = Serial.readStringUntil('\n');
        bool isSpacePress = (rawInput.indexOf(' ') >= 0);
        rawInput.trim();

        // SPACE BAR: Toggle gripper open/closed
        if (isSpacePress && rawInput.length() == 0) {
            gripperOverride = true;
            gripperOpen = !gripperOpen;
            Serial.printf(">> Gripper: %s\n", gripperOpen ? "OPEN (90 deg)" : "CLOSED (0 deg)");
        }
        // "roll,pitch": Manual servo override
        else if (rawInput.length() > 0) {
            int commaIndex = rawInput.indexOf(',');
            if (commaIndex > 0) {
                float targetRoll = rawInput.substring(0, commaIndex).toFloat();
                float targetPitch = rawInput.substring(commaIndex + 1).toFloat();
                
                Serial.printf("Test Input -> Target Roll: %.2f | Target Pitch: %.2f\n", targetRoll, targetPitch);
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

        // Override gripper roll if spacebar toggle is active
        if (gripperOverride) {
            roll = gripperOpen ? -6.0f : -25.0f;  // -6 = 90° servo (open), -25 = 0° servo (closed)
        }

        // Update servos with IMU data (Max 50Hz / 20ms)
        static unsigned long lastServoUpdate = 0;
        if (millis() - lastServoUpdate >= 20) {
            lastServoUpdate = millis();
            servoController.update(roll, pitch, yaw);
        }

        // Print debug data every 50ms 
        if (millis() - lastPrintTime >= 50) {
            lastPrintTime = millis();
            Serial.printf("Target [Roll: %5.1f | Pitch: %5.1f | Yaw: %5.1f] ---> Servos [Gripper: %3d | Elevation: %3d | Yaw: %3d]\n",
                          roll, pitch, yaw,
                          servoController.getGripperAngle(), servoController.getElevationAngle(), servoController.getYawAngle());
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
