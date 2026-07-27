# IMU-Controlled Gripper Arm

An ESP32-based robotic gripper arm that uses a **BNO085 9-DOF IMU** for real-time attitude estimation and maps the resulting orientation angles (roll, pitch, yaw) to three servo motors (gripper, elevation, yaw ).

Built with [PlatformIO](https://platformio.org/) and the Arduino framework.

---

## Repository Structure

```
imu-gripper-arm/
├── include/              # Global project headers
│   └── config.h          # Calibration matrices, magnetic references, filter thresholds
│
├── src/                  # Application entry points
│   └── main.cpp              # Full arm control (IMU → attitude → 3 servos)
│
├── lib/                  # Project-local libraries
│   ├── IMUSensor/            # Hardware abstraction layer
│   │   ├── IMUSensor.h           # Abstract IMU interface (accel, gyro, mag)
│   │   ├── BNO085_HAL.h/.cpp     # Concrete BNO085 driver (I²C, SH-2 reports)
│   │
│   ├── AttitudeEstimator/    # Sensor fusion / orientation estimation
│   │   ├── AttitudeEstimator.h/.cpp  # Strategy manager
│   │   ├── MadgwickFilter.h/.cpp     # Madgwick gradient-descent AHRS (not used in the project)
│   │   ├── MahonyFilter.h/.cpp       # Mahony complementary AHRS (not used in the project)
│   │   ├── ESKFFilter.h/.cpp         # Error-State Kalman Filter (used in the project)
│   │   ├── MEKF.h/.cpp              # Multiplicative Extended Kalman Filter (not used in the project)
│   │   └── NoneFilter.h/.cpp        # Pass-through (raw quaternion from accel+mag) (not used in the project)
│   │
│   ├── SensorCalibration/    # Offline calibration routines
│   │   ├── SensorCalibration.h/.cpp  # Ellipsoid fit, static offset, noise variance,
│   │                                 # Allan variance collection, tumble calibration
│   │
│   └── ServoController/      # Servo output stage
│       ├── ServoController.h/.cpp    # LPF smoothing, deadband, slew-rate limiting,
│                                     # IMU-to-servo angle mapping
│
├── scripts/              # Python helper scripts (host-side)
│   ├── collect_serial.py         # Collects raw gyro data over serial → CSV
│   └── analyze_allan_variance.py # Allan deviation analysis & plotting
│
├── platformio.ini        # Build environment & dependencies
├── teleplot_full.json    # Teleplot layout for real-time serial plotting visualization 
├── .gitignore
└── README.md            
```

---

## Build & Upload

The project compiles the `full_arm` target by default.

Build & upload:

```bash
pio run -t upload
pio device monitor          # 115200 baud
```

---

## Hardware

| Component | Details |
|---|---|
| **MCU** | ESP32 DevKit V1 |
| **IMU** | BNO085 (I²C, SDA 21 / SCL 22) |
| **Servos** | 3× PWM servos — Gripper (GPIO 32), Elevation (GPIO 33), Yaw (GPIO 27) |

---

## Libraries & Dependencies

### PlatformIO 
- `Adafruit BNO08x` — SH-2 sensor hub driver
- `ArduinoEigen` — Eigen linear algebra (used in Kalman filters & calibration)
- `ESP32Servo` — Hardware PWM servo control

### Project-local (`lib/`)

| Library | What it does |
|---|---|
| **IMUSensor** | Abstract sensor interface + BNO085 concrete driver |
| **AttitudeEstimator** | Strategy-pattern filter manager with 5 interchangeable filters (Madgwick, Mahony, ESKF, MEKF, None) |
| **SensorCalibration** | Ellipsoid-fit (accel/mag), static zero-rate offset (gyro), noise variance estimation, Allan variance data collection, tumble calibration |
| **ServoController** | Maps IMU angles to servo PWM with configurable LPF, deadband, slew-rate limiting, and input range mapping |

> [!NOTE]
> **Component Origin & Attribution:**
> - **`IMUSensor`**: Base sensor abstraction interface was imported from a previous project. The BNO085 driver was adapted from the Adafruit BNO08x library and developed for this project.
> - **`AttitudeEstimator`**: Estimator framework and baseline orientation filters (`MadgwickFilter`, `MahonyFilter`, `MEKF`, `NoneFilter`) were imported from a previous project. The **`ESKFFilter`** (Error-State Kalman Filter) was developed specifically for this project.

---

## Attitude Filters

Select a filter in code via:

```cpp
estimator.selectFilter(AttitudeFilterSel::ESKF);
```

| Filter | Description |
|---|---|
| `NONE` | Direct quaternion from accelerometer + magnetometer (no gyro fusion) |
| `MADGWICK` | Gradient-descent complementary filter |
| `MAHONY` | Proportional-integral complementary filter |
| `ESKF` | Error-State Kalman Filter (quaternion error-state, full 9-DOF) |
| `MEKF` | Multiplicative Extended Kalman Filter |

---

## Calibration Workflow

1. **Static calibration** — keep the sensor still to estimate gyro zero-rate offsets and noise variances
2. **Dynamic calibration** — rotate the sensor in all directions for accelerometer/magnetometer ellipsoid fit
3. **Tumble calibration** — multi-position static poses via serial prompt for high-precision accel/mag calibration
4. **Allan variance** — long-duration (e.g. 30 min) stationary recording for gyro noise characterisation

Results (offsets & gain matrices) are stored in [`include/config.h`](include/config.h).

### Python Scripts

```bash
# Collect raw gyro data over serial
python scripts/collect_serial.py --port COM3 --baud 115200 --out allan_data.csv

# Compute & plot Allan deviation
python scripts/analyze_allan_variance.py --csv allan_data.csv --rate 100
```

Requires: `pyserial`, `pandas`, `numpy`, `matplotlib`, `allantools`
