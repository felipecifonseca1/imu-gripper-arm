#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// --- Pin Definitions ---
#define I2C_SDA 21
#define I2C_SCL 22
#define BNO08X_INT 4 // Hardware Interrupt Pin

// --- Hardware Instances ---
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// --- Timing Variables ---
unsigned long lastPrintTime = 0;
unsigned long sampleCount = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n--- BNO085 Hardware Verification Test ---");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); 

    if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire, 0)) {
        Serial.println("ERROR: Failed to find BNO085! Check wiring and pull-up resistors.");
        while (1) { delay(10); }
    }
    Serial.println("BNO085 Initialized Successfully!");

    // Enable sensor reports at 100Hz (10,000 microseconds)
    if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) {
        Serial.println("Could not enable Accelerometer");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
        Serial.println("Could not enable Gyroscope");
    }
    if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000)) {
        Serial.println("Could not enable Magnetometer");
    }
}

void loop() {

    // Check if there is a new sensor event
    if (bno08x.getSensorEvent(&sensorValue)) {
        sampleCount++;

        if (millis() - lastPrintTime >= 500) {
            lastPrintTime = millis();

            switch (sensorValue.sensorId) {
                case SH2_ACCELEROMETER:
                    Serial.printf("ACCEL (m/s2) | X: %.2f | Y: %.2f | Z: %.2f\n",
                                  sensorValue.un.accelerometer.x,
                                  sensorValue.un.accelerometer.y,
                                  sensorValue.un.accelerometer.z);
                    break;
                    
                case SH2_GYROSCOPE_CALIBRATED:
                    Serial.printf("GYRO (rad/s) | X: %.2f | Y: %.2f | Z: %.2f\n",
                                  sensorValue.un.gyroscope.x,
                                  sensorValue.un.gyroscope.y,
                                  sensorValue.un.gyroscope.z);
                    break;

                case SH2_MAGNETIC_FIELD_CALIBRATED:
                    Serial.printf("MAG (uT)     | X: %.2f | Y: %.2f | Z: %.2f\n",
                                  sensorValue.un.magneticField.x,
                                  sensorValue.un.magneticField.y,
                                  sensorValue.un.magneticField.z);
                    Serial.println("---------------------------------------------------");
                    break;
            }
        }
    }
}