#include <Arduino.h>

#include "mpu6050.h"

#include "imu_filter.h"


constexpr uint32_t IMU_INTERVAL_MS = 10;
constexpr uint32_t PRINT_INTERVAL_MS = 100;

ImuFilter imuFilter(2.0f);

uint32_t lastImuTime = 0;
uint32_t lastPrintTime = 0;

Mpu6050Data data;
EulerAngles latestAngles;


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

        Serial.println("Orientation filter initialized");
    }

    lastImuTime = millis();
    lastPrintTime = lastImuTime;
}


void loop()
{
    uint32_t currentTime = millis();

    if (currentTime - lastImuTime >= IMU_INTERVAL_MS)
    {
        float deltaTime = (currentTime - lastImuTime) / 1000.0f;

        lastImuTime = currentTime;

        if (mpu6050Read(data))
        {
            imuFilter.update(data, deltaTime);

            latestAngles = imuFilter.getEulerAngles();
        }
        else
        {
            Serial.println("MPU6050 read failed");
        }
    }

    if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)
        {
            lastPrintTime = currentTime;

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


            Mpu6050Orientation orientation;

            mpu6050GetAccelOrientation(
                data,
                orientation
            );

            Serial.print("ROLL/PITCH [deg]: ");

            Serial.print(orientation.roll, 3);

            Serial.print(", ");

            Serial.println(orientation.pitch, 3);


            Serial.print(
                "ROLL/PITCH/YAW [deg]: "
            );

            Serial.print(latestAngles.roll, 3);
            Serial.print(", ");

            Serial.print(latestAngles.pitch, 3);
            Serial.print(", ");

            Serial.println(latestAngles.yaw, 3);

            Serial.println();
        }
}