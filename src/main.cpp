#include <Arduino.h>

#include "mpu6050.h"

#include "orientation_filter.h"


uint32_t lastReadTime = 0;

constexpr uint32_t READ_INTERVAL_MS = 100;

ComplementaryFilter orientationFilter;

uint32_t lastFilterTime = 0;


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

    Mpu6050Data initialData;

    if (mpu6050Read(initialData))
    {
        Mpu6050Orientation initialOrientation;

        mpu6050GetAccelOrientation(
            initialData,
            initialOrientation
        );

        orientationFilter.reset(
            initialOrientation.roll,
            initialOrientation.pitch
        );

        Serial.println("Orientation filter initialized");
    }

    lastFilterTime = millis();
}


void loop()
{
    uint32_t currentTime = millis();

    if (currentTime - lastReadTime >= READ_INTERVAL_MS)
    {
        float deltaTime = (currentTime - lastFilterTime) / 1000.0f;

        lastFilterTime = currentTime;
        lastReadTime = currentTime;

        Mpu6050Data data;

        if (mpu6050Read(data))
        {
            OrientationEstimate orientationEstimate =
                orientationFilter.update(data, deltaTime);

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

            // Serial.println();


            Mpu6050Orientation orientation;

            mpu6050GetAccelOrientation(
                data,
                orientation
            );

            Serial.print("FILTERED ROLL/PITCH [deg]: ");

            Serial.print(orientation.roll, 3);

            Serial.print(", ");

            Serial.println(orientation.pitch, 3);

            Serial.println();
        }
        else
        {
            Serial.println("MPU6050 read failed");
        }
    }
}