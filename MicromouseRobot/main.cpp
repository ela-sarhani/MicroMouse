#include <Arduino.h>
#include "Motors.h"
#include "Sensors.h"
#include "PIDController.h"
#include "config.h"

uint32_t lastRunTime = 0;

void setup() {
    Serial.begin(115200);
    Sensors::init();
    Sensors::calibrateGyro();
    Motors::init();
    Motion::init();

    Serial.println("Time_ms,TargetYaw,CurrentYaw,YawError,MeasVelL,MeasVelR,TargetVelL,TargetVelR");
    lastRunTime = millis();
}

void loop() {
    uint32_t now = millis();

    // Drive forward 1000 mm every 5 seconds
    if (now - lastRunTime >= 5000) {
        lastRunTime = now;
        Motion::moveForward(1000.0f, 150.0f);
    }
}