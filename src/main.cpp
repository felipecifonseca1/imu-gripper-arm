#include <Arduino.h>
#include <Wire.h>
#include "BNO085_HAL.h"
#include "AttitudeEstimator.h"
#include "SensorCalibration.h"

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



void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n--- BNO085 Hardware Verification Test ---");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); 
    
    if (!imu.init(true, false)) {
        Serial.println("ERROR: Failed to initialize BNO085!");
        while (1) { delay(10); }
    }
    Serial.println("BNO085 Initialized Successfully!");
    
    // SensorCalibration::runDynamicCalibration(&imu); // Fast Accel/Mag scale and offsets (Rotate sensor)
    // SensorCalibration::runTumbleCalibration(&imu);  // High-Precision Accel/Mag (Static poses via Serial 's')
    // SensorCalibration::runStaticCalibration(&imu);  // For Gyro offset and Noise variances (Keep still)
    // SensorCalibration::runAllanVarianceCollection(&imu, 1800000); // 30min test at 100Hz

    estimator.selectFilter(AttitudeFilterSel::ESKF);
    estimator.setUseMagnetometer(true);
}

void loop() {
    // Sensor data acquisition 
    if (imu.update()) {
        
        // Calculate the exact dt for the Kalman Filter
        unsigned long currentMicros = micros();
        float dt = (currentMicros - lastSampleTime) / 1000000.0f;
        lastSampleTime = currentMicros;

        // Declare temporary holding vectors
        float ax = imu.getAccX();
        float ay = imu.getAccY();
        float az = imu.getAccZ();
        float gx = imu.getGyroX_rads();
        float gy = imu.getGyroY_rads();
        float gz = imu.getGyroZ_rads();
        float mx = imu.getMagX();
        float my = imu.getMagY();
        float mz = imu.getMagZ();

        // Feed the data into the wrapper 
        estimator.update(dt, false);

        // Extract the filtered angles
        float roll = estimator.getRoll();
        float pitch = estimator.getPitch();
        float yaw = estimator.getYaw();

        // Print debug data every 50ms
        if (millis() - lastPrintTime >= 50) {
            lastPrintTime = millis();
            
            Serial.printf(
                "Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | ax: %6.2f | ay: %6.2f | az: %6.2f\n"
                ">Roll:%f\n>Pitch:%f\n>Yaw:%f\n"
                ">ax:%f\n>ay:%f\n>az:%f\n"
                ">gx:%f\n>gy:%f\n>gz:%f\n"
                ">mx:%f\n>my:%f\n>mz:%f\n",
                roll, pitch, yaw, ax, ay, az,
                roll, pitch, yaw,
                ax, ay, az,
                gx, gy, gz,
                mx, my, mz
            );
        }
    }
}