#include "mpu6050.h"

#include <Wire.h>
#include <math.h>


constexpr uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;

constexpr float ACCEL_SENSITIVITY = 16384.0f;
constexpr float GYRO_SENSITIVITY = 131.0f;


static Mpu6050GyroBias gyroBias =
{
    0.0f,
    0.0f,
    0.0f
};


static int16_t combineBytes(uint8_t highByte,
                            uint8_t lowByte)
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(highByte) << 8) |
        lowByte
    );
}


bool mpu6050Init()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00);

    uint8_t error = Wire.endTransmission();

    return error == 0;
}


bool mpu6050ReadRaw(Mpu6050RawData& data)
{
    // Tell MPU6050 which register to start reading from
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_ACCEL_XOUT_H);

    // Keep control of the I2C bus for the following read
    uint8_t error = Wire.endTransmission(false);

    if (error != 0)
    {
        return false;
    }


    // Request AX through GZ = 14 bytes
    uint8_t bytesRequested = 14;

    uint8_t bytesReceived =
        Wire.requestFrom(
            MPU6050_ADDRESS,
            bytesRequested,
            true
        );

    if (bytesReceived != bytesRequested)
    {
        return false;
    }


    uint8_t buffer[14];

    for (uint8_t i = 0; i < 14; i++)
    {
        buffer[i] = Wire.read();
    }


    data.ax = combineBytes(buffer[0], buffer[1]);
    data.ay = combineBytes(buffer[2], buffer[3]);
    data.az = combineBytes(buffer[4], buffer[5]);

    data.temperature =
        combineBytes(buffer[6], buffer[7]);

    data.gx = combineBytes(buffer[8], buffer[9]);
    data.gy = combineBytes(buffer[10], buffer[11]);
    data.gz = combineBytes(buffer[12], buffer[13]);

    return true;
}


bool mpu6050Read(Mpu6050Data& data)
{
    Mpu6050RawData rawData;

    if (!mpu6050ReadRaw(rawData))
    {
        return false;
    }

    data.ax =
        static_cast<float>(rawData.ax) /
        ACCEL_SENSITIVITY;

    data.ay =
        static_cast<float>(rawData.ay) /
        ACCEL_SENSITIVITY;

    data.az =
        static_cast<float>(rawData.az) /
        ACCEL_SENSITIVITY;


    data.gx =
        static_cast<float>(rawData.gx) /
        GYRO_SENSITIVITY -
        gyroBias.gx;

    data.gy =
        static_cast<float>(rawData.gy) /
        GYRO_SENSITIVITY -
        gyroBias.gy;

    data.gz =
        static_cast<float>(rawData.gz) /
        GYRO_SENSITIVITY -
        gyroBias.gz;

    return true;
}


bool mpu6050CalibrateGyro(uint16_t sampleCount)
{
    if (sampleCount == 0)
    {
        return false;
    }

    float sumGx = 0.0f;
    float sumGy = 0.0f;
    float sumGz = 0.0f;

    uint16_t successfulSamples = 0;

    for (uint16_t i = 0; i < sampleCount; i++)
    {
        Mpu6050RawData rawData;

        if (mpu6050ReadRaw(rawData))
        {
            float gx =
                static_cast<float>(rawData.gx) /
                GYRO_SENSITIVITY;

            float gy =
                static_cast<float>(rawData.gy) /
                GYRO_SENSITIVITY;

            float gz =
                static_cast<float>(rawData.gz) /
                GYRO_SENSITIVITY;

            sumGx += gx;
            sumGy += gy;
            sumGz += gz;

            successfulSamples++;
        }

        delay(5);
    }

    if (successfulSamples == 0)
    {
        return false;
    }

    gyroBias.gx = sumGx / successfulSamples;
    gyroBias.gy = sumGy / successfulSamples;
    gyroBias.gz = sumGz / successfulSamples;

    return true;
}


Mpu6050GyroBias mpu6050GetGyroBias()
{
    return gyroBias;
}


bool mpu6050GetAccelOrientation(
    const Mpu6050Data& data,
    Mpu6050Orientation& orientation)
{

    orientation.roll =
        atan2f(data.ay, data.az) *
        RAD_TO_DEG;

    float denominator =
        sqrtf(
            data.ay * data.ay +
            data.az * data.az
        );

    orientation.pitch =
        atan2f(-data.ax, denominator) *
        RAD_TO_DEG;

    return true;
}