#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>
#include "config.h"

// Generic PID controller with anti-windup (integral clamped to output range)
// and output saturation.
class PIDController {
public:
  PIDController(float kp, float ki, float kd, float outMin, float outMax);

  float compute(float setpoint, float measurement, float dtSeconds);
  void  reset();
  void  setGains(float kp, float ki, float kd);
  void  setOutputLimits(float outMin, float outMax);

private:
  float _kp, _ki, _kd;
  float _outMin, _outMax;
  float _integral;
  float _prevError;
  bool  _firstRun;
};

// -----------------------------------------------------------------------------
// Motion: higher-level motion primitives built on top of Motors + Sensors
// (distance PID, heading-hold PID, wall-centering PID, in-place turn PID).
// These are blocking calls (they run their own control loop internally using
// millis()) which keeps FSM/Floodfill code simple: "move one cell", "turn 90".
// -----------------------------------------------------------------------------
class Motion {
public:
  static void init();

  // Drives forward `distanceMM` while holding heading (and, if walls are
  // present on both sides, centering between them). Returns false on timeout.
  static bool moveForward(float distanceMM, float targetSpeedMMS);

  // Rotates in place by `deltaDegrees` (positive = clockwise / right turn)
  // relative to current heading, using the IMU yaw PID. Returns false on timeout.
  static bool turnInPlace(float deltaDegrees);

  static void stopMotion();

private:
  static PIDController velPidL;
  static PIDController velPidR;
  static PIDController headingPid;
  static PIDController turnPid;
  static PIDController wallCenterPid;
};

#endif // PIDCONTROLLER_H
