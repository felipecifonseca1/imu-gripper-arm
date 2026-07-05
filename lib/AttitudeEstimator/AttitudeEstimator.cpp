#include "AttitudeEstimator.h"
#include <math.h>
#include "../../include/config.h"

/**
 * @brief Initialize the estimator with an IMU pointer.
 * @param imu Pointer to an IMUSensor instance.
 */
AttitudeEstimator::AttitudeEstimator(IMUSensor* imu) : _imu(imu) {
    _activeFilter = &_madgwick; // Default
    setMagneticLocation(DEFAULT_MAG_LOCATION);
    resetOrientation(); 
}

/**
 * @brief Main update loop. Pulls raw data from IMU and updates the active filter.
 * @param dt Time delta in seconds.
 */
void AttitudeEstimator::update(float dt, bool ignoreAccel) {
    if (!_imu) return; 
    _deltaT = dt;

    float ax_raw = _imu->getAccX();
    float ay_raw = _imu->getAccY();
    float az_raw = _imu->getAccZ();
    
    float ax = ACCEL_GAIN[0] * (ax_raw - ACCEL_OFFSET[0]) + ACCEL_GAIN[1] * (ay_raw - ACCEL_OFFSET[1]) + ACCEL_GAIN[2] * (az_raw - ACCEL_OFFSET[2]);
    float ay = ACCEL_GAIN[3] * (ax_raw - ACCEL_OFFSET[0]) + ACCEL_GAIN[4] * (ay_raw - ACCEL_OFFSET[1]) + ACCEL_GAIN[5] * (az_raw - ACCEL_OFFSET[2]);
    float az = ACCEL_GAIN[6] * (ax_raw - ACCEL_OFFSET[0]) + ACCEL_GAIN[7] * (ay_raw - ACCEL_OFFSET[1]) + ACCEL_GAIN[8] * (az_raw - ACCEL_OFFSET[2]);
    
    _transformedAccX = ax;
    _transformedAccY = ay;
    _transformedAccZ = az;

    // Automatic Accel Masking: Ignore if beyond thresholds
    float a_norm = sqrt(ax * ax + ay * ay + az * az);
    if (a_norm > ORIENTATION_MASK_MAX_G || a_norm < ORIENTATION_MASK_MIN_G) {
        ignoreAccel = true;
    }

    float gx_raw = _imu->getGyroX_rads();
    float gy_raw = _imu->getGyroY_rads();
    float gz_raw = _imu->getGyroZ_rads();
    
    float gx = GYRO_GAIN[0] * (gx_raw - GYRO_OFFSET[0]) + GYRO_GAIN[1] * (gy_raw - GYRO_OFFSET[1]) + GYRO_GAIN[2] * (gz_raw - GYRO_OFFSET[2]);
    float gy = GYRO_GAIN[3] * (gx_raw - GYRO_OFFSET[0]) + GYRO_GAIN[4] * (gy_raw - GYRO_OFFSET[1]) + GYRO_GAIN[5] * (gz_raw - GYRO_OFFSET[2]);
    float gz = GYRO_GAIN[6] * (gx_raw - GYRO_OFFSET[0]) + GYRO_GAIN[7] * (gy_raw - GYRO_OFFSET[1]) + GYRO_GAIN[8] * (gz_raw - GYRO_OFFSET[2]);

    // Apply Gyro Cutoff 
    float cutoff_rads = ATTITUDE_GYRO_CUTOFF_DPS * DEG_TO_RAD;
    if (abs(gx) < cutoff_rads) gx = 0.0f;
    if (abs(gy) < cutoff_rads) gy = 0.0f;
    if (abs(gz) < cutoff_rads) gz = 0.0f;

    // Store transformed gyro values for telemetry
    _transformedGyroX = gx * RAD_TO_DEG;
    _transformedGyroY = gy * RAD_TO_DEG;
    _transformedGyroZ = gz * RAD_TO_DEG;

    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    if (_useMagnetometer) {
        float mx_raw = _imu->getMagX(); 
        float my_raw = _imu->getMagY(); 
        float mz_raw = _imu->getMagZ(); 
        
        mx = MAG_GAIN[0] * (mx_raw - MAG_OFFSET[0]) + MAG_GAIN[1] * (my_raw - MAG_OFFSET[1]) + MAG_GAIN[2] * (mz_raw - MAG_OFFSET[2]);
        my = MAG_GAIN[3] * (mx_raw - MAG_OFFSET[0]) + MAG_GAIN[4] * (my_raw - MAG_OFFSET[1]) + MAG_GAIN[5] * (mz_raw - MAG_OFFSET[2]);
        mz = MAG_GAIN[6] * (mx_raw - MAG_OFFSET[0]) + MAG_GAIN[7] * (my_raw - MAG_OFFSET[1]) + MAG_GAIN[8] * (mz_raw - MAG_OFFSET[2]);

        _transformedMagX = mx;
        _transformedMagY = my;
        _transformedMagZ = mz;
    }

    if (_filterSel == AttitudeFilterSel::NAV_MEKF) {
        // Handled externally in FlightController to allow 15-state fusion
        return;
    }

    if (_activeFilter) {
        _activeFilter->update(dt, ax, ay, az, gx, gy, gz, mx, my, mz, ignoreAccel);
    }
}

/**
 * @brief Select which orientation filter to use.
 * @param sel Enum value for NONE, MADGWICK, or MAHONY.
 */
void AttitudeEstimator::selectFilter(AttitudeFilterSel sel) {
    _filterSel = sel;
    switch (sel) {
        case AttitudeFilterSel::MADGWICK:
            _activeFilter = &_madgwick;
            break;
        case AttitudeFilterSel::MAHONY:
            _activeFilter = &_mahony;
            break;
        case AttitudeFilterSel::MEKF:
            _activeFilter = &_mekf;
            break;
        case AttitudeFilterSel::NONE:
            _activeFilter = &_noneFilter;
            break;
        case AttitudeFilterSel::ESKF:
            _activeFilter = &_eskf;
            break;
        case AttitudeFilterSel::NAV_MEKF:
            // For NAV_MEKF, there is no internal active filter since it's external.
            _activeFilter = nullptr;
            break;
    }
}

/**
 * @brief Enable or disable the zeta-based drift learning for Madgwick filter.
 * @param enabled True to enable.
 */
void AttitudeEstimator::setDriftLearning(bool enabled) {
    _madgwick.setDriftLearning(enabled);
}

/**
 * @brief Set the filter's beta parameter.
 * @param errorDegPerSec Estimated gyroscope drift in degrees per second.
 */
void AttitudeEstimator::setFilterBeta(float errorDegPerSec) {
    _madgwick.setFilterBeta(errorDegPerSec);
    // You could also map this to Mahony's Kp here if desired.
}

/**
 * @brief Resets the internal orientation quaternion.
 */
void AttitudeEstimator::resetOrientation() {
    _madgwick.reset();
    _mahony.reset();
    _mekf.reset();
    _noneFilter.reset();
    _eskf.reset();
}

/**
 * @brief Manually sets the internal quaternion and normalizes it.
 */
void AttitudeEstimator::setQuaternion(float w, float x, float y, float z) {
    if (_activeFilter) {
        _activeFilter->setQuaternion(w, x, y, z);
    }
}

void AttitudeEstimator::resetEstimatorState() {
    resetOrientation();
}

// --- Euler Angle Computations ---
float AttitudeEstimator::getRoll()  const { return computeRoll(); }
float AttitudeEstimator::getPitch() const { return computePitch();}
float AttitudeEstimator::getYaw()   const { return computeYaw();  }

float AttitudeEstimator::getQuaternionW() const { 
    float w, x, y, z; 
    if (_activeFilter) _activeFilter->getQuaternion(w, x, y, z); 
    else { w = 1.0f; } // fallback
    return w; 
}
float AttitudeEstimator::getQuaternionX() const { 
    float w, x, y, z; 
    if (_activeFilter) _activeFilter->getQuaternion(w, x, y, z); 
    else { x = 0.0f; } // fallback
    return x; 
}
float AttitudeEstimator::getQuaternionY() const { 
    float w, x, y, z; 
    if (_activeFilter) _activeFilter->getQuaternion(w, x, y, z); 
    else { y = 0.0f; } // fallback
    return y; 
}
float AttitudeEstimator::getQuaternionZ() const { 
    float w, x, y, z; 
    if (_activeFilter) _activeFilter->getQuaternion(w, x, y, z); 
    else { z = 0.0f; } // fallback
    return z; 
}

/**
 * @brief Computes Roll from current quaternion.
 * @return Roll in degrees.
 */
float AttitudeEstimator::computeRoll() const {
    float w = getQuaternionW(); float x = getQuaternionX(); float y = getQuaternionY(); float z = getQuaternionZ();
    float roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    return roll * RAD_TO_DEG;
}

/**
 * @brief Computes Pitch from current quaternion.
 * @return Pitch in degrees.
 */
float AttitudeEstimator::computePitch() const {
    float w = getQuaternionW(); float x = getQuaternionX(); float y = getQuaternionY(); float z = getQuaternionZ();
    float sinp = 2.0f * (w * y - z * x);
    if (abs(sinp) >= 1) {
        return copysignf(M_PI / 2.0f, sinp) * RAD_TO_DEG; 
    } else {
        return asinf(sinp) * RAD_TO_DEG;
    }
}

/**
 * @brief Computes Yaw from current quaternion.
 * @return Yaw in degrees.
 */
float AttitudeEstimator::computeYaw() const {
    float w = getQuaternionW(); float x = getQuaternionX(); float y = getQuaternionY(); float z = getQuaternionZ();
    float yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    return yaw * RAD_TO_DEG;
}

// --- Advanced Math Getters ---

/**
 * @brief Calculates the tilt angle relative to the earth's gravity vector.
 * @param physicalZAxisDown True if the IMU is mounted Z-axis pointing down.
 * @return Tilt angle in degrees [0-180].
 */
float AttitudeEstimator::getTilt() const {
    float qw = getQuaternionW();
    float qx = getQuaternionX();
    float qy = getQuaternionY();
    float qz = getQuaternionZ();

    // Standard Z-Up "upward" vector projected into body: [2(q1q3 - q0q2), 2(q0q1 + q2q3), q0^2 - q1^2 - q2^2 + q3^2]
    float cos_tilt = qw * qw - qx * qx - qy * qy + qz * qz;
    if (cos_tilt > 1.0f) cos_tilt = 1.0f;
    else if (cos_tilt < -1.0f) cos_tilt = -1.0f;
    
    // We return standard tilt because data is already standardized in the HAL
    return acosf(cos_tilt) * RAD_TO_DEG;
}

/**
 * @brief Computes Z-axis acceleration in the world frame with gravity removed.
 * @details Uses the current quaternion to rotate the body-frame acceleration 
 *          to the world frame. Note: On pad, this should be close to 0.0.
 * @return Vertical acceleration in m/s^2.
 */
float AttitudeEstimator::getNetVerticalAcceleration() const {
    float qw = getQuaternionW(); float qx = getQuaternionX(); float qy = getQuaternionY(); float qz = getQuaternionZ();

    float worldZAcceleration =
        2.0f * (qx * qz - qw * qy) * _transformedAccX +
        2.0f * (qw * qx + qy * qz) * _transformedAccY +
        (qw * qw - qx * qx - qy * qy + qz * qz) * _transformedAccZ;

    float netG = (worldZAcceleration - 1.0f);
    float netAcc = netG * _G_GRAVITY;
    
    // Apply noise floor (threshold) to keep pad telemetry clean
    if (abs(netAcc) < NET_ACC_THRESHOLD) return 0.0f;

    return netAcc;
}


void AttitudeEstimator::setMEKFTuning(float q_proc, float r_accel, float r_mag) {
    _mekf.setMEKFTuning(q_proc, r_accel, r_mag);
}

void AttitudeEstimator::setMagneticReference(float mx, float my, float mz) {
    float norm = sqrt(mx*mx + my*my + mz*mz);
    if (norm > 1e-6f) {
        _mag_ref_x = mx / norm;
        _mag_ref_y = my / norm;
        _mag_ref_z = mz / norm;
        
        _mekf.setMagneticReference(_mag_ref_x, _mag_ref_y, _mag_ref_z);
    }
}

void AttitudeEstimator::setMagneticLocation(MagLocation location) {
    switch(location) {
        case MagLocation::SAO_PAULO:
            setMagneticReference(0.724f, -0.289f, 0.626f); // Declination: -21.76° | Inclination: 38.77°
            break;
        case MagLocation::PIRASSUNUNGA:
            setMagneticReference(0.730f, -0.283f, 0.622f); // Declination: -21.22° | Inclination: 38.48°
            break;
        case MagLocation::MUNICH:
            setMagneticReference(0.418f, 0.014f, -0.908f); // Declination: 1.94° | Inclination: 65.26°
            break;
        case MagLocation::MIDLAND_TX:
            setMagneticReference(0.497f, 0.056f, -0.866f); // Declination: 6.43° | Inclination: 59.99°
            break;
        default:
            // Default to horizontal North if unknown
            setMagneticReference(1.0f, 0.0f, 0.0f);
            break;
    }
}

void AttitudeEstimator::setMagnetometerWeight(float weight) {
    _madgwick.setMagnetometerWeight(weight);
    _mahony.setMagnetometerWeight(weight);
}

/**
 * @brief Enable or disable magnetometer fusion.
 * @param use True to enable.
 */
void AttitudeEstimator::setUseMagnetometer(bool use) {
    _useMagnetometer = use;
    _mekf.setUseMagnetometer(use);
}
