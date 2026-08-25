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

        Mpu6050RawData data;

        if (mpu6050ReadRaw(data))
        {
            Serial.print("ACC: ");

            Serial.print(data.ax);
            Serial.print(", ");

            Serial.print(data.ay);
            Serial.print(", ");

            Serial.println(data.az);


            Serial.print("GYRO: ");

            Serial.print(data.gx);
            Serial.print(", ");

            Serial.print(data.gy);
            Serial.print(", ");

            Serial.println(data.gz);

            Serial.println();
        }
        else
        {
            Serial.println("MPU6050 read failed");
        }
    }
}