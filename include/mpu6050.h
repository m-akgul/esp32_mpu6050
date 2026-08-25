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


bool mpu6050Init();

bool mpu6050ReadRaw(Mpu6050RawData& data);

#endif