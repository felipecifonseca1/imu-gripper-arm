#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <Arduino.h>
#include "IMUSensor.h"
#include "../../include/config.h"

/**
 * @brief Abstract base class (Strategy) for orientation filters.
 */
class IAttitudeFilter {
public:
    /**
     * @brief Virtual destructor for attitude filter strategy interface.
     */
    virtual ~IAttitudeFilter() = default;

    /**
     * @brief Updates filter algorithm with sensor observations.
     * @param dt Integration time step in seconds.
     * @param ax Accelerometer X in g.
     * @param ay Accelerometer Y in g.
     * @param az Accelerometer Z in g.
     * @param gx Gyroscope X in rad/s.
     * @param gy Gyroscope Y in rad/s.
     * @param gz Gyroscope Z in rad/s.
     * @param mx Magnetometer X in uT.
     * @param my Magnetometer Y in uT.
     * @param mz Magnetometer Z in uT.
     * @param ignoreAccel Flag to ignore accelerometer correction during dynamic spikes.
     */
    virtual void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) = 0;

    /**
     * @brief Gets current estimated unit quaternion.
     * @param w Output w scalar component reference.
     * @param x Output x vector component reference.
     * @param y Output y vector component reference.
     * @param z Output z vector component reference.
     */
    virtual void getQuaternion(float& w, float& x, float& y, float& z) const = 0;

    /**
     * @brief Sets internal unit quaternion state.
     * @param w Scalar w component.
     * @param x Vector x component.
     * @param y Vector y component.
     * @param z Vector z component.
     */
    virtual void setQuaternion(float w, float x, float y, float z) = 0;

    /**
     * @brief Resets filter internal states and covariances to defaults.
     */
    virtual void reset() = 0;
};

#include "MadgwickFilter.h"
#include "MahonyFilter.h"
#include "MEKF.h"
#include "NoneFilter.h"
#include "ESKFFilter.h"

/**
 * @enum AttitudeFilterSel
 * @brief Available filters for attitude estimation.
 */
enum class AttitudeFilterSel {
    NONE,     
    MADGWICK, 
    MAHONY,   
    ESKF,     
    MEKF,     
    NAV_MEKF, 
};

class AttitudeEstimator {
    public:
        /**
         * @brief Constructs AttitudeEstimator attached to an IMU sensor source.
         * @param imu Pointer to IMUSensor instance.
         */
        AttitudeEstimator(IMUSensor* imu);

        /**
         * @brief Main sensor update and fusion pipeline.
         * @param dt Integration time step in seconds.
         * @param ignoreAccel Flag to suppress accelerometer update.
         */
        void update(float dt, bool ignoreAccel = false);

        /**
         * @brief Selects active attitude estimation filter strategy.
         * @param sel Enum selection value for filter algorithm.
         */
        void selectFilter(AttitudeFilterSel sel);

        /**
         * @brief Gets currently selected filter strategy enum.
         * @return Currently active AttitudeFilterSel value.
         */
        AttitudeFilterSel getCurrentFilter() const { return _filterSel; }
        
        /**
         * @brief Enables or disables drift learning mode for Madgwick filter.
         * @param enabled True to enable drift estimation.
         */
        void setDriftLearning(bool enabled);

        /**
         * @brief Sets filter beta gain based on estimated gyro error.
         * @param errorDegPerSec Gyroscope error standard deviation in deg/s.
         */
        void setFilterBeta(float errorDegPerSec);

        /**
         * @brief Resets orientation quaternions across all filter instances.
         */
        void resetOrientation();

        /**
         * @brief Resets total estimator state to baseline defaults.
         */
        void resetEstimatorState();

        /**
         * @brief Sets orientation quaternion for active filter.
         * @param w Quaternion scalar w component.
         * @param x Quaternion vector x component.
         * @param y Quaternion vector y component.
         * @param z Quaternion vector z component.
         */
        void setQuaternion(float w, float x, float y, float z);

        /**
         * @brief Gets estimated Roll angle.
         * @return Roll angle in degrees.
         */
        float getRoll() const;

        /**
         * @brief Gets estimated Pitch angle.
         * @return Pitch angle in degrees.
         */
        float getPitch() const;

        /**
         * @brief Gets estimated Yaw angle.
         * @return Yaw angle in degrees.
         */
        float getYaw() const;
        
        /**
         * @brief Gets unit quaternion W component.
         * @return Scalar W component.
         */
        float getQuaternionW() const;

        /**
         * @brief Gets unit quaternion X component.
         * @return Vector X component.
         */
        float getQuaternionX() const;

        /**
         * @brief Gets unit quaternion Y component.
         * @return Vector Y component.
         */
        float getQuaternionY() const;

        /**
         * @brief Gets unit quaternion Z component.
         * @return Vector Z component.
         */
        float getQuaternionZ() const;

        /**
         * @brief Computes tilt angle relative to local vertical.
         * @return Tilt angle in degrees [0-180].
         */
        float getTilt() const;

        /**
         * @brief Computes net vertical acceleration with gravity removed.
         * @return Vertical acceleration in m/s^2.
         */
        float getNetVerticalAcceleration() const;

        /**
         * @brief Gets transformed accelerometer X value after gain and offset calibration.
         * @return Accelerometer X in g.
         */
        float getTransformedAccX() const { return _transformedAccX; }

        /**
         * @brief Gets transformed accelerometer Y value after gain and offset calibration.
         * @return Accelerometer Y in g.
         */
        float getTransformedAccY() const { return _transformedAccY; }

        /**
         * @brief Gets transformed accelerometer Z value after gain and offset calibration.
         * @return Accelerometer Z in g.
         */
        float getTransformedAccZ() const { return _transformedAccZ; }
        
        /**
         * @brief Gets transformed gyroscope X value after gain and offset calibration.
         * @return Gyroscope X rate in deg/s.
         */
        float getTransformedGyroX() const { return _transformedGyroX; }

        /**
         * @brief Gets transformed gyroscope Y value after gain and offset calibration.
         * @return Gyroscope Y rate in deg/s.
         */
        float getTransformedGyroY() const { return _transformedGyroY; }

        /**
         * @brief Gets transformed gyroscope Z value after gain and offset calibration.
         * @return Gyroscope Z rate in deg/s.
         */
        float getTransformedGyroZ() const { return _transformedGyroZ; }

        /**
         * @brief Gets transformed magnetometer X value after gain and offset calibration.
         * @return Magnetometer X flux in uT.
         */
        float getTransformedMagX() const { return _transformedMagX; }

        /**
         * @brief Gets transformed magnetometer Y value after gain and offset calibration.
         * @return Magnetometer Y flux in uT.
         */
        float getTransformedMagY() const { return _transformedMagY; }

        /**
         * @brief Gets transformed magnetometer Z value after gain and offset calibration.
         * @return Magnetometer Z flux in uT.
         */
        float getTransformedMagZ() const { return _transformedMagZ; }

        /**
         * @brief Enables or disables magnetometer fusion.
         * @param use True to use magnetometer measurements.
         */
        void setUseMagnetometer(bool use);

        /**
         * @brief Sets magnetometer fusion weight factor.
         * @param weight Magnetometer weight between 0.0 and 1.0.
         */
        void setMagnetometerWeight(float weight);

        /**
         * @brief Checks if magnetometer fusion is enabled.
         * @return True if magnetometer fusion is enabled, false otherwise.
         */
        bool getUseMagnetometer() const { return _useMagnetometer; }

        /**
         * @brief Reads local normalized magnetic reference vector components.
         * @param mx Output reference X component.
         * @param my Output reference Y component.
         * @param mz Output reference Z component.
         */
        void getMagneticReference(float &mx, float &my, float &mz) const { mx = _mag_ref_x; my = _mag_ref_y; mz = _mag_ref_z; }
        
        /**
         * @brief Gets attached IMUSensor pointer.
         * @return Pointer to IMUSensor instance.
         */
        IMUSensor* getIMU() const { return _imu; }

        /**
         * @brief Configures noise tuning parameters for Extended Kalman Filter.
         * @param q_proc Process noise parameter.
         * @param r_accel Accelerometer measurement noise variance.
         * @param r_mag Magnetometer measurement noise variance.
         */
        void setMEKFTuning(float q_proc, float r_accel, float r_mag);

        /**
         * @brief Sets custom normalized magnetic reference vector.
         * @param mx Magnetometer reference X vector component.
         * @param my Magnetometer reference Y vector component.
         * @param mz Magnetometer reference Z vector component.
         */
        void setMagneticReference(float mx, float my, float mz);

        /**
         * @brief Sets magnetic reference vector according to predefined geographical location.
         * @param location MagLocation enum value.
         */
        void setMagneticLocation(MagLocation location);

    private:
        IMUSensor* _imu;
        bool _useMagnetometer = true;
        
        AttitudeFilterSel _filterSel = AttitudeFilterSel::MADGWICK;
        float _deltaT = 0.0f;

        float _mag_ref_x = 1.0f; 
        float _mag_ref_y = 0.0f;
        float _mag_ref_z = 0.0f;

        static constexpr float _G_GRAVITY = 9.80665f;

        float computeRoll() const;
        float computePitch() const;
        float computeYaw() const;

        float _transformedAccX = 0.0f;
        float _transformedAccY = 0.0f;
        float _transformedAccZ = 0.0f;

        float _transformedGyroX = 0.0f;
        float _transformedGyroY = 0.0f;
        float _transformedGyroZ = 0.0f;

        float _transformedMagX = 0.0f;
        float _transformedMagY = 0.0f;
        float _transformedMagZ = 0.0f;

        MadgwickFilter _madgwick;
        MahonyFilter _mahony;
        MEKF _mekf;
        NoneFilter _noneFilter;
        ESKFFilter _eskf;
        
        IAttitudeFilter* _activeFilter;
};

#endif // ATTITUDE_ESTIMATOR_H
