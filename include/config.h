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

#endif // CONFIG_H
