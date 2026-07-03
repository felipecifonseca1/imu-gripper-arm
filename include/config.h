#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

enum class MagLocation : uint8_t {
    SAO_PAULO,
    PIRASSUNUNGA,
    MUNICH,
    MIDLAND_TX
};

constexpr MagLocation DEFAULT_MAG_LOCATION = MagLocation::MUNICH;
constexpr float ORIENTATION_MASK_MAX_G = 1.5f;
constexpr float ORIENTATION_MASK_MIN_G = 0.5f;
constexpr float ATTITUDE_GYRO_CUTOFF_DPS = 0.2f;
constexpr float NET_ACC_THRESHOLD = 0.15f;

// --- Calibration Matrices (Default to Identity / Zero) ---
// Run SensorCalibration and replace these with the output
constexpr float ACCEL_OFFSET[3] = {0.0f, 0.0f, 0.0f};
constexpr float ACCEL_GAIN[9]   = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

constexpr float GYRO_OFFSET[3]  = {0.0f, 0.0f, 0.0f};
constexpr float GYRO_GAIN[9]    = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

constexpr float MAG_OFFSET[3]   = {0.0f, 0.0f, 0.0f};
constexpr float MAG_GAIN[9]     = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
#endif // CONFIG_H
