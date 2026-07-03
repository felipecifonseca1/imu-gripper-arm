#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <math.h>
#include <Arduino.h>

class MadgwickFilter : public IAttitudeFilter {
public:
    MadgwickFilter();

    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;
    void getQuaternion(float& w, float& x, float& y, float& z) const override;
    void setQuaternion(float w, float x, float y, float z) override;
    void reset() override;

    void setFilterBeta(float errorDegPerSec);
    void setDriftLearning(bool enabled);
    void setMagnetometerWeight(float weight);

private:
    float _q[4];
    
    float _beta;
    float _zeta;
    bool _useZeta;
    float _w_bx, _w_by, _w_bz;
    float _magWeight;
};

#endif // MADGWICK_FILTER_H
