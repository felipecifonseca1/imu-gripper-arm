#ifndef ESKF_FILTER_H
#define ESKF_FILTER_H

#include <math.h>

class ESKFFilter : public IAttitudeFilter {
public:
    ESKFFilter();

    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel) override;
    void getQuaternion(float& w, float& x, float& y, float& z) const override;
    void setQuaternion(float w, float x, float y, float z) override;
    void reset() override;

private:
    float _q[4];
};

#endif // ESKF_FILTER_H
