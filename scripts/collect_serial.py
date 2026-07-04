import serial
import csv
import time
import argparse

def collect_data(port, baudrate, output_file):
    print(f"Connecting to {port} at {baudrate} baud...")
    
    with serial.Serial(port, baudrate, timeout=1) as ser:
        print("Waiting for data stream... (Reset board if stuck)")
        
        # Flush input buffer to clear old data
        ser.reset_input_buffer()
        
        # Wait for start signal or direct data lines
        collecting = False
        with open(output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time_ms', 'gx_rads', 'gy_rads', 'gz_rads'])
            
            count = 0
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                if line == "END_ALLAN_DATA":
                    print("\nEnd signal received. Data collection complete.")
                    break
                
                if not collecting:
                    if line == "START_ALLAN_DATA":
                        print("Start signal received. Collecting data...")
                        collecting = True
                        continue
                    # Check if it looks like data: "timestamp,gx,gy,gz"
                    parts = line.split(',')
                    if len(parts) == 4 and parts[0].isdigit():
                        print("Detected data stream. Collecting data...")
                        collecting = True
                        # Don't continue, write this line
                
                if collecting:
                    try:
                        parts = line.split(',')
                        if len(parts) == 4:
                            writer.writerow(parts)
                            count += 1
                            if count % 1000 == 0:
                                print(f"\rCollected {count} samples...", end="", flush=True)
                    except ValueError:
                        print(f"\nSkipped malformed line: {line}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Collect Allan Variance data from serial")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--out", type=str, default="allan_variance_data.csv", help="Output CSV file")
    
    args = parser.parse_args()
    collect_data(args.port, args.baud, args.out)
