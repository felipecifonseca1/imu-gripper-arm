#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <Arduino.h>
#include "IMUSensor.h"

/**
 * @brief Abstract base class (Strategy) for orientation filters.
 */
class IAttitudeFilter {
public:
    virtual ~IAttitudeFilter() = default;

    virtual void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) = 0;
    virtual void getQuaternion(float& w, float& x, float& y, float& z) const = 0;
    virtual void setQuaternion(float w, float x, float y, float z) = 0;
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
        AttitudeEstimator(IMUSensor* imu);
        void update(float dt, bool ignoreAccel = false);

        void selectFilter(AttitudeFilterSel sel);
        AttitudeFilterSel getCurrentFilter() const { return _filterSel; }
        
        void setDriftLearning(bool enabled);
        void setFilterBeta(float errorDegPerSec);
        void resetOrientation();
        void resetEstimatorState();
        void setQuaternion(float w, float x, float y, float z);

        float getRoll() const;  
        float getPitch() const; 
        float getYaw() const;   
        
        float getQuaternionW() const; 
        float getQuaternionX() const; 
        float getQuaternionY() const; 
        float getQuaternionZ() const; 

        float getTilt() const;
        float getNetVerticalAcceleration() const;

        float getTransformedAccX() const { return _transformedAccX; }
        float getTransformedAccY() const { return _transformedAccY; }
        float getTransformedAccZ() const { return _transformedAccZ; }
        
        float getTransformedGyroX() const { return _transformedGyroX; }
        float getTransformedGyroY() const { return _transformedGyroY; }
        float getTransformedGyroZ() const { return _transformedGyroZ; }

        void setUseMagnetometer(bool use);
        void setMagnetometerWeight(float weight);
        bool getUseMagnetometer() const { return _useMagnetometer; }
        void getMagneticReference(float &mx, float &my, float &mz) const { mx = _mag_ref_x; my = _mag_ref_y; mz = _mag_ref_z; }
        
        IMUSensor* getIMU() const { return _imu; }
        void setMEKFTuning(float q_proc, float r_accel, float r_mag);
        void setMagneticReference(float mx, float my, float mz);
        void setMagneticLocation(uint8_t location);

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

        MadgwickFilter _madgwick;
        MahonyFilter _mahony;
        MEKF _mekf;
        NoneFilter _noneFilter;
        ESKFFilter _eskf;
        
        IAttitudeFilter* _activeFilter;
};

#endif // ATTITUDE_ESTIMATOR_H
