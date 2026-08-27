#include "imu_filter.h"

#include <math.h>


ImuFilter::ImuFilter(float proportionalGain)
    : kp(proportionalGain)
{
    reset();
}


void ImuFilter::reset()
{
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
}


void ImuFilter::update(
    const Mpu6050Data& data,
    float deltaTime)
{
    // Gyroscope values must be in radians/second
    float gx = data.gx * PI / 180.0f;
    float gy = data.gy * PI / 180.0f;
    float gz = data.gz * PI / 180.0f;


    // Accelerometer measurement
    float ax = data.ax;
    float ay = data.ay;
    float az = data.az;


    // Normalize accelerometer vector
    float accelMagnitude =
        sqrtf(ax * ax + ay * ay + az * az);

    if (accelMagnitude > 0.0001f)
    {
        ax /= accelMagnitude;
        ay /= accelMagnitude;
        az /= accelMagnitude;


        // Estimated gravity direction from quaternion
        float estimatedGx =
            2.0f * (q.x * q.z - q.w * q.y);

        float estimatedGy =
            2.0f * (q.w * q.x + q.y * q.z);

        float estimatedGz =
            q.w * q.w -
            q.x * q.x -
            q.y * q.y +
            q.z * q.z;


        // Error between measured and estimated gravity
        float errorX =
            ay * estimatedGz -
            az * estimatedGy;

        float errorY =
            az * estimatedGx -
            ax * estimatedGz;

        float errorZ =
            ax * estimatedGy -
            ay * estimatedGx;


        // Correct gyroscope
        gx += kp * errorX;
        gy += kp * errorY;
        gz += kp * errorZ;
    }


    // Quaternion derivative
    float qDotW =
        0.5f *
        (-q.x * gx - q.y * gy - q.z * gz);

    float qDotX =
        0.5f *
        ( q.w * gx + q.y * gz - q.z * gy);

    float qDotY =
        0.5f *
        ( q.w * gy - q.x * gz + q.z * gx);

    float qDotZ =
        0.5f *
        ( q.w * gz + q.x * gy - q.y * gx);


    // Integrate quaternion
    q.w += qDotW * deltaTime;
    q.x += qDotX * deltaTime;
    q.y += qDotY * deltaTime;
    q.z += qDotZ * deltaTime;


    // Normalize quaternion
    float quaternionMagnitude =
        sqrtf(
            q.w * q.w +
            q.x * q.x +
            q.y * q.y +
            q.z * q.z
        );

    if (quaternionMagnitude > 0.0001f)
    {
        q.w /= quaternionMagnitude;
        q.x /= quaternionMagnitude;
        q.y /= quaternionMagnitude;
        q.z /= quaternionMagnitude;
    }
}


Quaternion ImuFilter::getQuaternion() const
{
    return q;
}


EulerAngles ImuFilter::getEulerAngles() const
{
    EulerAngles angles;


    // Roll
    float rollNumerator =
        2.0f *
        (q.w * q.x + q.y * q.z);

    float rollDenominator =
        1.0f -
        2.0f *
        (q.x * q.x + q.y * q.y);

    angles.roll =
        atan2f(
            rollNumerator,
            rollDenominator
        ) * RAD_TO_DEG;


    // Pitch
    float pitchValue =
        2.0f *
        (q.w * q.y - q.z * q.x);

    if (fabsf(pitchValue) >= 1.0f)
    {
        angles.pitch =
            copysignf(
                90.0f,
                pitchValue
            );
    }
    else
    {
        angles.pitch =
            asinf(pitchValue) *
            RAD_TO_DEG;
    }


    // Yaw
    float yawNumerator =
        2.0f *
        (q.w * q.z + q.x * q.y);

    float yawDenominator =
        1.0f -
        2.0f *
        (q.y * q.y + q.z * q.z);

    angles.yaw =
        atan2f(
            yawNumerator,
            yawDenominator
        ) * RAD_TO_DEG;


    return angles;
}