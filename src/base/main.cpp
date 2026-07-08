#include <Arduino.h>
#include "ServoController.h"

// --- Pin Definitions ---
#define GRIPPER_PIN 26
#define ELEVATION_PIN 27

#define HC05_RX 16
#define HC05_TX 17

// --- Servo Controller Instance ---
ServoController servoController;

void setup() {
    Serial.begin(115200); // For PC Serial Monitor testing
    Serial2.begin(115200, SERIAL_8N1, HC05_RX, HC05_TX); // For HC-05 Bluetooth

    servoController.init(GRIPPER_PIN, ELEVATION_PIN);
    
    // Optional: Tune smoothing and deadband
    // servoController.setFilterAlpha(0.2);
    // servoController.setDeadband(2.0);

    Serial.println("\n--- Base Receiver Setup Complete ---");
    Serial.println("Send 'roll,pitch' via Serial or Bluetooth to test.");
    Serial.println("Example: '45,-30'");
}

void loop() {
    // --- 1. Read from USB Serial Monitor (testing without IMU) ---
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            int commaIndex = input.indexOf(',');
            if (commaIndex > 0) {
                float targetRoll = input.substring(0, commaIndex).toFloat();
                float targetPitch = input.substring(commaIndex + 1).toFloat();
                
                Serial.printf("Test Input -> Target Roll: %.2f | Target Pitch: %.2f\n", targetRoll, targetPitch);
                // When sending manual commands, we loop a bit to let the lowpass filter reach the target
                for(int i=0; i<50; i++) {
                    servoController.update(targetRoll, targetPitch);
                    delay(20);
                }
            }
        }
    }

    // --- 2. Read from Bluetooth (when connected to Wrist IMU) ---
    if (Serial2.available()) {
        String input = Serial2.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            int commaIndex = input.indexOf(',');
            if (commaIndex > 0) {
                float targetRoll = input.substring(0, commaIndex).toFloat();
                float targetPitch = input.substring(commaIndex + 1).toFloat();
                
                servoController.update(targetRoll, targetPitch);
            }
        }
    }
    
    // Slight delay for loop stability
    delay(20); 
}
