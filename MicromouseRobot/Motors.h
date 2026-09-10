#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

// Low-level differential-drive motor + encoder driver.
// Handles TB6612FNG PWM/direction pins and quadrature encoder ISRs.
// Velocity control (PID) lives in PIDController / Motion, not here.
class Motors {
public:
  static void init();

  // Raw PWM control, range [-PWM_MAX_DUTY, +PWM_MAX_DUTY]. Sign = direction.
  static void setLeftPWM(int16_t pwm);
  static void setRightPWM(int16_t pwm);
  static void setPWM(int16_t leftPwm, int16_t rightPwm);

  static void stop();   // coast (outputs disabled via STBY / hi-Z)
  static void brake();  // active short-brake both motors

  static void resetEncoders();
  static long getLeftCount();
  static long getRightCount();

  // Must be called at a fixed period (see config.h MOTION_LOOP_DT_MS) to
  // compute instantaneous wheel velocities in mm/s.
  static void updateVelocity(float dtSeconds);
  static float getLeftVelocityMMS();
  static float getRightVelocityMMS();

  static float getLeftDistanceMM();
  static float getRightDistanceMM();
  static float getAverageDistanceMM();

private:
  static volatile long encoderCountL;
  static volatile long encoderCountR;
  static long lastEncoderCountL;
  static long lastEncoderCountR;

  static float velocityL_mms;
  static float velocityR_mms;

  static void IRAM_ATTR isrEncoderL_A();
  static void IRAM_ATTR isrEncoderL_B();
  static void IRAM_ATTR isrEncoderR_A();
  static void IRAM_ATTR isrEncoderR_B();
};

#endif // MOTORS_H
