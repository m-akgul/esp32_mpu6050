#ifndef ORIENTATION_FILTER_H
#define ORIENTATION_FILTER_H

#include "mpu6050.h"


struct OrientationEstimate
{
    float roll;
    float pitch;
};


class ComplementaryFilter
{
public:
    ComplementaryFilter(float alpha = 0.98f);

    void reset(float roll, float pitch);

    OrientationEstimate update(
        const Mpu6050Data& data,
        float deltaTime
    );

private:
    float alpha;

    float roll;
    float pitch;
};

#endif