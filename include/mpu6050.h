#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

constexpr uint8_t MPU6050_ADDRESS = 0x68;

struct Mpu6050RawData
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temperature;

    int16_t gx;
    int16_t gy;
    int16_t gz;
};

struct Mpu6050Data
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
};

struct Mpu6050GyroBias
{
    float gx;
    float gy;
    float gz;
};

struct Mpu6050Orientation
{
    float roll;
    float pitch;
};


bool mpu6050Init();

bool mpu6050ReadRaw(Mpu6050RawData& data);

bool mpu6050Read(Mpu6050Data& data);

bool mpu6050CalibrateGyro(uint16_t sampleCount);

Mpu6050GyroBias mpu6050GetGyroBias();

bool mpu6050GetAccelOrientation(
    const Mpu6050Data& data,
    Mpu6050Orientation& orientation
);

#endif