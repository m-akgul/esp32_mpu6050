#include "orientation_filter.h"

#include <math.h>


ComplementaryFilter::ComplementaryFilter(float alpha)
    : alpha(alpha),
      roll(0.0f),
      pitch(0.0f)
{
}


void ComplementaryFilter::reset(
    float initialRoll,
    float initialPitch)
{
    roll = initialRoll;
    pitch = initialPitch;
}


OrientationEstimate ComplementaryFilter::update(
    const Mpu6050Data& data,
    float deltaTime)
{

    // Accelerometer orientation
    float accelRoll =
        atan2f(data.ay, data.az) *
        RAD_TO_DEG;

    float denominator =
        sqrtf(
            data.ay * data.ay +
            data.az * data.az
        );

    float accelPitch =
        atan2f(-data.ax, denominator) *
        RAD_TO_DEG;


    // Gyroscope prediction
    float gyroRoll =
        roll +
        data.gx * deltaTime;

    float gyroPitch =
        pitch +
        data.gy * deltaTime;


    // Complementary filter
    roll =
        alpha * gyroRoll +
        (1.0f - alpha) * accelRoll;

    pitch =
        alpha * gyroPitch +
        (1.0f - alpha) * accelPitch;


    OrientationEstimate result;

    result.roll = roll;
    result.pitch = pitch;

    return result;
}