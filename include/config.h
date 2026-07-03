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
constexpr float ACCEL_OFFSET[3] = {-0.014878f, 0.022074f, -0.025961f};
constexpr float ACCEL_GAIN[9] = {1.037306f, 0.000000f, 0.000000f, 0.000000f, 0.998076f, 0.000000f, 0.000000f, 0.000000f, 0.985926f};

constexpr float GYRO_OFFSET[3] = {0.000049f, 0.000150f, 0.000088f};
constexpr float GYRO_GAIN[9]    = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

constexpr float MAG_OFFSET[3] = {3.890188f, -4.042345f, 1.342559f};
constexpr float MAG_GAIN[9] = {0.027269f, 0.000000f, 0.000000f, 0.000000f, 0.017655f, 0.000000f, 0.000000f, 0.000000f, 0.019616f};
#endif // CONFIG_H
