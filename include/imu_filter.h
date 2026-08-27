#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "mpu6050.h"


struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};


struct EulerAngles
{
    float roll;
    float pitch;
    float yaw;
};


class ImuFilter
{
public:
    ImuFilter(float proportionalGain = 2.0f);

    void reset();

    void update(
        const Mpu6050Data& data,
        float deltaTime
    );

    Quaternion getQuaternion() const;

    EulerAngles getEulerAngles() const;

private:
    float kp;

    Quaternion q;
};

#endif