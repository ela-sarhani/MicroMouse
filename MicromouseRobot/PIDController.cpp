#include "PIDController.h"
#include "Motors.h"
#include "Sensors.h"

// =============================================================================
//  PIDController
// =============================================================================
PIDController::PIDController(float kp, float ki, float kd, float outMin, float outMax)
  : _kp(kp), _ki(ki), _kd(kd), _outMin(outMin), _outMax(outMax),
    _integral(0.0f), _prevError(0.0f), _firstRun(true) {}

void PIDController::setGains(float kp, float ki, float kd) {
  _kp = kp; _ki = ki; _kd = kd;
}

void PIDController::setOutputLimits(float outMin, float outMax) {
  _outMin = outMin; _outMax = outMax;
}

void PIDController::reset() {
  _integral = 0.0f;
  _prevError = 0.0f;
  _firstRun = true;
}

float PIDController::compute(float setpoint, float measurement, float dtSeconds) {
  if (dtSeconds <= 0.0f) dtSeconds = 0.001f;

  float error = setpoint - measurement;

  // Proportional term
  float pTerm = _kp * error;

  // Integral accumulation with conditional anti-windup.
  float candidateIntegral = _integral + error * dtSeconds;

  if (_ki > 0.0f) {
    float minIntegral = _outMin / _ki;
    float maxIntegral = _outMax / _ki;
    if (minIntegral > maxIntegral) {
      float tmp = minIntegral;
      minIntegral = maxIntegral;
      maxIntegral = tmp;
    }
    candidateIntegral = constrain(candidateIntegral, minIntegral, maxIntegral);
  } else {
    candidateIntegral = 0.0f;
  }

  // Derivative term
  float derivative = _firstRun ? 0.0f : (error - _prevError) / dtSeconds;
  float dTerm = _kd * derivative;

  float output = pTerm + (_ki * candidateIntegral) + dTerm;

  if (_ki > 0.0f) {
    if ((output > _outMax && error > 0.0f) || (output < _outMin && error < 0.0f)) {
      candidateIntegral = _integral;
      output = pTerm + (_ki * candidateIntegral) + dTerm;
    }
  }

  _integral = candidateIntegral;
  _prevError = error;
  _firstRun = false;

  return constrain(output, _outMin, _outMax);
}

// =============================================================================
//  Motion primitives
// =============================================================================
PIDController Motion::velPidL(VEL_PID_KP, VEL_PID_KI, VEL_PID_KD, VEL_PID_OUT_MIN, VEL_PID_OUT_MAX);
PIDController Motion::velPidR(VEL_PID_KP, VEL_PID_KI, VEL_PID_KD, VEL_PID_OUT_MIN, VEL_PID_OUT_MAX);
PIDController Motion::headingPid(HEADING_PID_KP, HEADING_PID_KI, HEADING_PID_KD, HEADING_PID_OUT_MIN, HEADING_PID_OUT_MAX);
PIDController Motion::turnPid(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD, TURN_PID_OUT_MIN, TURN_PID_OUT_MAX);
PIDController Motion::wallCenterPid(WALL_CENTER_KP, WALL_CENTER_KI, WALL_CENTER_KD, WALL_CENTER_OUT_MIN, WALL_CENTER_OUT_MAX);

void Motion::init() {
  velPidL.reset();
  velPidR.reset();
  headingPid.reset();
  turnPid.reset();
  wallCenterPid.reset();
}

void Motion::stopMotion() {
  // Direct pin level brake
  Motors::brake();

  velPidL.reset();
  velPidR.reset();
  headingPid.reset();
  turnPid.reset();
  wallCenterPid.reset();
  
  Motors::resetEncoders();
  
  delay(100); // Allow physical momentum to dissipate under active brake
  
  // Coast to off state
  Motors::stop();
}

bool Motion::moveForward(float distanceMM, float targetSpeedMMS) {
  Motors::resetEncoders();
  velPidL.reset();
  velPidR.reset();
  headingPid.reset();
  wallCenterPid.reset();

  // Clamp commanded speed to physical motor capability to avoid perpetual
  // saturation of the velocity PID when the setpoint is unreachable.
  if (targetSpeedMMS > MAX_MOTOR_MMS) targetSpeedMMS = MAX_MOTOR_MMS;

  float startYaw = Sensors::getYaw();
  unsigned long startTime = millis();
  unsigned long lastLoop = micros();

  while (Motors::getAverageDistanceMM() < distanceMM) {
    if (millis() - startTime > CELL_MOVE_TIMEOUT_MS) {
      stopMotion();
      return false;
    }

    unsigned long now = micros();
    float dt = (now - lastLoop) / 1000000.0f;
    if (dt < (MOTION_LOOP_DT_MS / 1000.0f)) continue; // hold to fixed loop rate
    lastLoop = now;

    Sensors::updateIMU();
    Sensors::update();
    Motors::updateVelocity(dt); // Must be updated before PID compute!

    float remaining = distanceMM - Motors::getAverageDistanceMM();
    float speedCmd = targetSpeedMMS;
    if (remaining < 60.0f) {
      speedCmd = max(80.0f, targetSpeedMMS * (remaining / 60.0f));
    }

    float yawError = startYaw - Sensors::getYaw();
    float headingCorrection = headingPid.compute(0.0f, -yawError, dt);

    float centerCorrection = 0.0f;
    if (Sensors::wallLeft() && Sensors::wallRight()) {
      float sideError = (float)Sensors::getR() - (float)Sensors::getL();
      centerCorrection = wallCenterPid.compute(0.0f, sideError, dt);
    }

    float trim = headingCorrection + centerCorrection;
    float targetL = max(0.0f, speedCmd - trim);
    float targetR = max(0.0f, speedCmd + trim);

    float outL = velPidL.compute(targetL, Motors::getLeftVelocityMMS(), dt);
    float outR = velPidR.compute(targetR, Motors::getRightVelocityMMS(), dt);

    Motors::setPWM((int16_t)outL, (int16_t)outR);
  }

  stopMotion();
  return true;
}

bool Motion::turnInPlace(float deltaDegrees) {
  turnPid.reset();
  velPidL.reset();
  velPidR.reset();

  float startYaw = Sensors::getYaw();
  float targetYaw = startYaw + deltaDegrees;
  // normalize target into [-180, 180)
  while (targetYaw >= 180.0f) targetYaw -= 360.0f;
  while (targetYaw < -180.0f) targetYaw += 360.0f;

  unsigned long startTime = millis();
  unsigned long lastLoop = micros();
  unsigned long withinToleranceSince = 0;

  while (true) {
    if (millis() - startTime > TURN_TIMEOUT_MS) {
      stopMotion();
      return false;
    }

    unsigned long now = micros();
    float dt = (now - lastLoop) / 1000000.0f;
    if (dt < (MOTION_LOOP_DT_MS / 1000.0f)) continue;
    lastLoop = now;

    Sensors::updateIMU();

    float currentYaw = Sensors::getYaw();
    float error = targetYaw - currentYaw;
    while (error >= 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    if (fabs(error) <= TURN_DONE_TOLERANCE_DEG) {
      if (withinToleranceSince == 0) withinToleranceSince = millis();
      if (millis() - withinToleranceSince >= TURN_DONE_STABLE_MS) break;
    } else {
      withinToleranceSince = 0;
    }

    Motors::updateVelocity(dt);
    float turnSpeed = turnPid.compute(0.0f, -error, dt); // mm/s differential
    // Clamp turnSpeed to per-wheel linear maximum to avoid asking the
    // velocity PID for impossible wheel speeds.
    if (turnSpeed > MAX_MOTOR_MMS) turnSpeed = MAX_MOTOR_MMS;
    if (turnSpeed < -MAX_MOTOR_MMS) turnSpeed = -MAX_MOTOR_MMS;
    float outL = velPidL.compute(-turnSpeed, Motors::getLeftVelocityMMS(), dt);
    float outR = velPidR.compute(turnSpeed, Motors::getRightVelocityMMS(), dt);

    Motors::setPWM((int16_t)outL, (int16_t)outR);
  }

  stopMotion();
  Sensors::resetYaw(targetYaw); // re-anchor to avoid slow drift accumulation
  return true;
}
