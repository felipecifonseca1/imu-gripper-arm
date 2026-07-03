#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include <math.h>

class MahonyFilter : public IAttitudeFilter {
public:
    MahonyFilter();

    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;
    void getQuaternion(float& w, float& x, float& y, float& z) const override;
    void setQuaternion(float w, float x, float y, float z) override;
    void reset() override;

    void setMagnetometerWeight(float weight);
    void setGains(float kp, float ki);

private:
    float _q[4];
    
    float _Kp;
    float _Ki;
    float _ix, _iy, _iz; // Integral feedback terms
    float _magWeight;
};

#endif // MAHONY_FILTER_H
