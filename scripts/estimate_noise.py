#!/usr/bin/env python3
import serial
import time
import numpy as np
import argparse
import sys

# To use this script, temporarily change your main.cpp loop to print raw values:
# Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, mx, my, mz);

def collect_data(port, baudrate, samples):
    print(f"Connecting to {port} at {baudrate} baud...")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except Exception as e:
        print(f"Failed to connect: {e}")
        sys.exit(1)

    print("Please leave the IMU completely stationary.")
    print("Flushing buffer...")
    time.sleep(2)
    ser.reset_input_buffer()

    data = []
    print(f"Collecting {samples} samples...")
    
    while len(data) < samples:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) == 9:
                vals = [float(p) for p in parts]
                data.append(vals)
                if len(data) % 100 == 0:
                    print(f"Progress: {len(data)}/{samples}")
        except Exception as e:
            # Ignore malformed lines
            pass

    ser.close()
    return np.array(data)

def main():
    parser = argparse.ArgumentParser(description="Estimate IMU sensor noise variance from stationary serial data.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--samples", type=int, default=1000, help="Number of samples to collect (default: 1000)")
    args = parser.parse_args()

    data = collect_data(args.port, args.baud, args.samples)
    
    # Calculate variance for each axis
    variances = np.var(data, axis=0)

    # Average the variance across X, Y, Z for each sensor type
    accel_var = np.mean(variances[0:3])
    gyro_var = np.mean(variances[3:6])
    mag_var = np.mean(variances[6:9])

    print("\n" + "="*50)
    print("CALIBRATION RESULTS (Paste this into ESKFFilter.cpp):")
    print("="*50)
    print(f"    const float AccelerometerNoise = {accel_var:.8e}f; // Measured")
    print(f"    const float GyroscopeNoise = {gyro_var:.8e}f; // Measured")
    print(f"    const float MagnetometerNoise = {mag_var:.8e}f; // Measured")
    print("="*50)
    print("NOTE: Ensure your LinearAccelerationNoise is slightly higher than AccelerometerNoise.")

if __name__ == "__main__":
    main()
