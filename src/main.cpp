#include <Arduino.h>

#include "mpu6050.h"


uint32_t lastReadTime = 0;

constexpr uint32_t READ_INTERVAL_MS = 100;


void setup()
{
    Serial.begin(115200);

    if (!mpu6050Init())
    {
        Serial.println("MPU6050 initialization failed");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("MPU6050 initialized successfully");

    Serial.println("Keep MPU6050 completely still...");
    Serial.println("Gyroscope calibration starting");

    if (mpu6050CalibrateGyro(500))
    {
        Mpu6050GyroBias bias =
            mpu6050GetGyroBias();

        Serial.println("Calibration complete");

        Serial.print("Gyro bias: ");
        Serial.print(bias.gx, 3);
        Serial.print(", ");

        Serial.print(bias.gy, 3);
        Serial.print(", ");

        Serial.println(bias.gz, 3);
    }
    else
    {
        Serial.println("Gyroscope calibration failed");

        while (true)
        {
            delay(1000);
        }
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