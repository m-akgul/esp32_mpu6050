#include <Arduino.h>
#include <Wire.h>

void setup()
{
    Serial.begin(115200);

    Wire.begin(21, 22);

    Serial.println();
    Serial.println("I2C scanner started");
}

void loop()
{
    uint8_t devicesFound = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);

        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("Device found at 0x");

            if (address < 16)
            {
                Serial.print("0");
            }

            Serial.println(address, HEX);

            devicesFound++;
        }
    }

    if (devicesFound == 0)
    {
        Serial.println("No I2C devices found");
    }
    else
    {
        Serial.println("Scan complete");
    }

    Serial.println();

    delay(2000);
}