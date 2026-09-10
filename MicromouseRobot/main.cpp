#include <Arduino.h>
#include "Motors.h"
#include "Sensors.h"
#include "PIDController.h"
#include "config.h"

uint32_t lastStepTime = 0;
bool turnDirection = true; // true = +90 deg, false = -90 deg

void setup() {
    Serial.begin(115200);
    Sensors::init();
    Sensors::calibrateGyro();
    Motors::init();
    Motion::init();

    // Headers matching the Serial Plotter fields
    Serial.println("Time_ms,TargetYaw,CurrentYaw,YawError,PWM_L,PWM_R");
    lastStepTime = millis();
}

void loop() {
    uint32_t now = millis();

    // Trigger a turn sequence every 4 seconds
    if (now - lastStepTime >= 4000) {
        lastStepTime = now;
        float targetAngle = turnDirection ? 90.0f : -90.0f;
        turnDirection = !turnDirection;

        // Perform in-place turn
        Motion::turnInPlace(targetAngle);
    }
}