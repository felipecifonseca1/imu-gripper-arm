#include <Arduino.h>
#include <Wire.h>
#include "BNO085_HAL.h"

// --- Pin Definitions ---
#define I2C_SDA 21
#define I2C_SCL 22
#define BNO08X_INT 4 // Hardware Interrupt Pin

// --- Hardware Instances ---
BNO085_HAL imu;

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

        //!TODO: Feed the data into the wrapper 
        // estimator.updateESKF(ax, ay, az, gx, gy, gz, mx, my, mz, false);

        //!TODO: Extract the filtered angles
        // float roll = estimator.computeRoll();
        // float pitch = estimator.computePitch();

        // Print debug data every 100ms
        if (millis() - lastPrintTime >= 100) {
            lastPrintTime = millis();
            Serial.printf("Acc [g]: %6.2f %6.2f %6.2f | Gyro [rad/s]: %6.2f %6.2f %6.2f | Mag [uT]: %6.2f %6.2f %6.2f\n",
                          ax, ay, az, gx, gy, gz, mx, my, mz);
        }
    }
}