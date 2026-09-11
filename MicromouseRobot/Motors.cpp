#include "Motors.h"

// ---- static member storage -------------------------------------------------
volatile long Motors::encoderCountL = 0;
volatile long Motors::encoderCountR = 0;
long Motors::lastEncoderCountL = 0;
long Motors::lastEncoderCountR = 0;
float Motors::velocityL_mms = 0.0f;
float Motors::velocityR_mms = 0.0f;

// Quadrature decode state: 2-bit (A<<1 | B) history per encoder, packed in a
// single volatile byte each. Standard "transition table" full 4x decode.
static volatile uint8_t stateL = 0;
static volatile uint8_t stateR = 0;

// Transition table indexed by (oldState<<2 | newState) -> +1 / -1 / 0
// Valid CW/CCW single-step transitions give +-1, invalid (noise/skip) give 0.
static const int8_t QUAD_TABLE[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

// -----------------------------------------------------------------------------
//  ENCODER ISRs
// -----------------------------------------------------------------------------
void IRAM_ATTR Motors::isrEncoderL_A() {
  uint8_t a = digitalRead(PIN_ENCODER_L_A);
  uint8_t b = digitalRead(PIN_ENCODER_L_B);
  uint8_t newState = (a << 1) | b;
  uint8_t idx = (stateL << 2) | newState;
  encoderCountL += QUAD_TABLE[idx & 0x0F];
  stateL = newState;
}
void IRAM_ATTR Motors::isrEncoderL_B() {
  isrEncoderL_A(); // same decode logic, either pin edge triggers re-sample
}
void IRAM_ATTR Motors::isrEncoderR_A() {
  uint8_t a = digitalRead(PIN_ENCODER_R_A);
  uint8_t b = digitalRead(PIN_ENCODER_R_B);
  uint8_t newState = (a << 1) | b;
  uint8_t idx = (stateR << 2) | newState;
  encoderCountR += QUAD_TABLE[idx & 0x0F];
  stateR = newState;
}
void IRAM_ATTR Motors::isrEncoderR_B() {
  isrEncoderR_A();
}

// -----------------------------------------------------------------------------

void Motors::init() {
  pinMode(PIN_MOTOR_L_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_L_AIN2, OUTPUT);
  pinMode(PIN_MOTOR_R_BIN1, OUTPUT);
  pinMode(PIN_MOTOR_R_BIN2, OUTPUT);
  pinMode(PIN_MOTOR_STBY,   OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, HIGH); // enable driver

  ledcAttach(PIN_MOTOR_L_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttach(PIN_MOTOR_R_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);

  pinMode(PIN_ENCODER_L_A, INPUT);
  pinMode(PIN_ENCODER_L_B, INPUT);
  pinMode(PIN_ENCODER_R_A, INPUT);
  pinMode(PIN_ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), isrEncoderL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_B), isrEncoderL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), isrEncoderR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_B), isrEncoderR_B, CHANGE);

  stop();
  resetEncoders();
}

void Motors::setLeftPWM(int16_t pwm) {
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  pwm = constrain(pwm, -PWM_MAX_DUTY, PWM_MAX_DUTY);
  if (pwm >= 0) {
    digitalWrite(PIN_MOTOR_L_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_L_AIN2, LOW);
  } else {
    digitalWrite(PIN_MOTOR_L_AIN1, LOW);
    digitalWrite(PIN_MOTOR_L_AIN2, HIGH);
    pwm = -pwm;
  }
  ledcWrite(PIN_MOTOR_L_PWM, pwm);
}

void Motors::setRightPWM(int16_t pwm) {
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  pwm = constrain(pwm, -PWM_MAX_DUTY, PWM_MAX_DUTY);
  if (pwm >= 0) {
    digitalWrite(PIN_MOTOR_R_BIN1, HIGH);
    digitalWrite(PIN_MOTOR_R_BIN2, LOW);
  } else {
    digitalWrite(PIN_MOTOR_R_BIN1, LOW);
    digitalWrite(PIN_MOTOR_R_BIN2, HIGH);
    pwm = -pwm;
  }
  ledcWrite(PIN_MOTOR_R_PWM, pwm);
}

void Motors::setPWM(int16_t leftPwm, int16_t rightPwm) {
  setLeftPWM(leftPwm);
  setRightPWM(rightPwm);
}

void Motors::stop() {
  digitalWrite(PIN_MOTOR_L_AIN1, LOW);
  digitalWrite(PIN_MOTOR_L_AIN2, LOW);
  digitalWrite(PIN_MOTOR_R_BIN1, LOW);
  digitalWrite(PIN_MOTOR_R_BIN2, LOW);
  ledcWrite(PIN_MOTOR_L_PWM, 0);
  ledcWrite(PIN_MOTOR_R_PWM, 0);
  digitalWrite(PIN_MOTOR_STBY, LOW);

  // Hard zero the velocity values so serial plotter drops to 0 immediately
  velocityL_mms = 0.0f;
  velocityR_mms = 0.0f;
}

void Motors::brake() {
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  digitalWrite(PIN_MOTOR_L_AIN1, HIGH);
  digitalWrite(PIN_MOTOR_L_AIN2, HIGH);
  digitalWrite(PIN_MOTOR_R_BIN1, HIGH);
  digitalWrite(PIN_MOTOR_R_BIN2, HIGH);
  ledcWrite(PIN_MOTOR_L_PWM, PWM_MAX_DUTY);
  ledcWrite(PIN_MOTOR_R_PWM, PWM_MAX_DUTY);
}

void Motors::resetEncoders() {
  noInterrupts();
  encoderCountL = 0;
  encoderCountR = 0;
  interrupts();
  lastEncoderCountL = 0;
  lastEncoderCountR = 0;
  velocityL_mms = 0.0f; // Clear static velocity variables
  velocityR_mms = 0.0f;
}

long Motors::getLeftCount() {
  noInterrupts();
  long v = encoderCountL;
  interrupts();
  return v;
}

long Motors::getRightCount() {
  noInterrupts();
  long v = encoderCountR;
  interrupts();
  return v;
}

// In Motors.cpp inside updateVelocity(float dt)
void Motors::updateVelocity(float dt) {
  if (dt <= 0.0001f) return;

  noInterrupts();
  long currentTicksL = encoderCountL;
  long currentTicksR = encoderCountR;
  interrupts();

  long deltaL = currentTicksL - lastEncoderCountL;
  long deltaR = currentTicksR - lastEncoderCountR;

  lastEncoderCountL = currentTicksL;
  lastEncoderCountR = currentTicksR;

  bool leftDisabled = (digitalRead(PIN_MOTOR_L_AIN1) == LOW) &&
                      (digitalRead(PIN_MOTOR_L_AIN2) == LOW);
  bool rightDisabled = (digitalRead(PIN_MOTOR_R_BIN1) == LOW) &&
                       (digitalRead(PIN_MOTOR_R_BIN2) == LOW);
  bool driverDisabled = (digitalRead(PIN_MOTOR_STBY) == LOW);

  if (leftDisabled || driverDisabled) {
    velocityL_mms = 0.0f;
    lastEncoderCountL = currentTicksL;
    deltaL = 0;
  }
  if (rightDisabled || driverDisabled) {
    velocityR_mms = 0.0f;
    lastEncoderCountR = currentTicksR;
    deltaR = 0;
  }

  float rawVelL = (deltaL * MM_PER_ENCODER_COUNT) / dt;
  float rawVelR = (-deltaR * MM_PER_ENCODER_COUNT) / dt;

  // Exponential Moving Average (EMA) Low-Pass Filter (alpha = 0.35)
  const float alpha = 0.35f;
  velocityL_mms = (alpha * rawVelL) + ((1.0f - alpha) * velocityL_mms);
  velocityR_mms = (alpha * rawVelR) + ((1.0f - alpha) * velocityR_mms);
}

float Motors::getLeftVelocityMMS()  { return velocityL_mms; }
float Motors::getRightVelocityMMS() { return velocityR_mms; }

float Motors::getLeftDistanceMM()  { return getLeftCount()  * MM_PER_ENCODER_COUNT; }
float Motors::getRightDistanceMM() { return -getRightCount() * MM_PER_ENCODER_COUNT; }
float Motors::getAverageDistanceMM() {
  return (getLeftDistanceMM() + getRightDistanceMM()) * 0.5f;
}