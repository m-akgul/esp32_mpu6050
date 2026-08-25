#include <Arduino.h>

#include "mpu6050.h"


uint32_t lastReadTime = 0;

constexpr uint32_t READ_INTERVAL_MS = 100;


void setup()
{
    Serial.begin(115200);

    bool success = mpu6050Init();

    if (success)
    {
        Serial.println("MPU6050 initialized successfully");
    }
    else
    {
        Serial.println("MPU6050 initialization failed");
    }
}


void loop()
{
    uint32_t currentTime = millis();

    if (currentTime - lastReadTime >= READ_INTERVAL_MS)
    {
        lastReadTime = currentTime;

        Mpu6050Data data;

        if (mpu6050Read(data))
        {
            Serial.print("ACC [g]: ");

            Serial.print(data.ax, 3);
            Serial.print(", ");

            Serial.print(data.ay, 3);
            Serial.print(", ");

            Serial.println(data.az, 3);


            Serial.print("GYRO [deg/s]: ");

            Serial.print(data.gx, 3);
            Serial.print(", ");

            Serial.print(data.gy, 3);
            Serial.print(", ");

            Serial.println(data.gz, 3);

            Serial.println();
        }
        else
        {
            Serial.println("MPU6050 read failed");
        }
    }
}