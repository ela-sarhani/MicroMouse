#include <Arduino.h>
#include "Motors.h"
#include "PIDController.h" // Includes your PID logic
#include "config.h"

const float TARGET_SPEED_MMS = SPEED_EXPLORE_MMS;
uint32_t lastLoopMicros = 0;
uint32_t stepStartTime = 0;

// Instantiate or reference your velocity PID controllers
PIDController pidLeft(VEL_PID_KP, VEL_PID_KI, VEL_PID_KD, -255.0f, 255.0f);
PIDController pidRight(VEL_PID_KP, VEL_PID_KI, VEL_PID_KD, -255.0f, 255.0f);

void setup() {
    Serial.begin(115200);
    Motors::init();
    Motors::resetEncoders();
    Serial.println("Rotate wheel exactly 1 full 360 degree turn by hand...");
    Serial.println("Time_ms,Target_mm_s,Left_mm_s,Right_mm_s");
    stepStartTime = millis();
    lastLoopMicros = micros();
}

void loop() {
    uint32_t currentMillis = millis();
    uint32_t currentMicros = micros();
    float dtSeconds = (currentMicros - lastLoopMicros) / 1000000.0f;

    if (dtSeconds < (MOTION_LOOP_DT_MS / 1000.0f)) {
        return;
    }

    lastLoopMicros = currentMicros;

    // 3-second motion step / 2-second pause
    uint32_t cycleTime = (currentMillis - stepStartTime) % 5000;
    float currentTarget = (cycleTime < 3000) ? TARGET_SPEED_MMS : 0.0f;

    // 1. Update encoder velocity calculations in Motors driver
    Motors::updateVelocity(dtSeconds);
    float leftVel  = Motors::getLeftVelocityMMS();
    float rightVel = Motors::getRightVelocityMMS();

    if (currentTarget <= 0.0f) {
        pidLeft.reset();
        pidRight.reset();
        Motors::stop();
    } else {
        // 2. Compute PID outputs (converts mm/s speed error -> PWM output)
        float pwmL = pidLeft.compute(currentTarget, leftVel, dtSeconds);
        float pwmR = pidRight.compute(currentTarget, rightVel, dtSeconds);

        // 3. Drive motors with raw computed PWM
        Motors::setPWM((int16_t)pwmL, (int16_t)pwmR);
    }

    // 4. Stream to Serial Plotter
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(currentTarget);
    Serial.print(",");
    Serial.print(leftVel);
    Serial.print(",");
    Serial.println(rightVel);
}