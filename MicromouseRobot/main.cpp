#include <Arduino.h>

#include "Motors.h"
#include "Sensors.h"
#include "PIDController.h"
#include "config.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println(" MPU + ENCODER STRAIGHT TEST");
    Serial.println("================================");

    // Initialize hardware
    Motors::init();
    Sensors::init();
    Motion::init();

    delay(1000);

    Serial.println("Keep robot straight on the floor.");
    Serial.println("Starting in 3 seconds...");

    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);

    Serial.println("START");

    // 1 meter forward
    bool result = Motion::moveForward(1000.0f, 80.0f);

    Motors::stop();

    Serial.println();
    Serial.println("================================");

    if (result)
        Serial.println("MOVE SUCCESS");
    else
        Serial.println("MOVE FAILED");

    Serial.println("================================");
}

void loop()
{
    // Nothing
}