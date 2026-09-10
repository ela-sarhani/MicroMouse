#ifndef FSM_H
#define FSM_H

#include <Arduino.h>
#include "config.h"

enum RobotState : uint8_t {
  STATE_CALIBRATION = 0,
  STATE_EXPLORATION = 1,
  STATE_WAIT        = 2,
  STATE_SPEED_RUN   = 3
};

class FSM {
public:
  static void init();
  static void update();   // call every loop() iteration
  static RobotState getState();

private:
  static RobotState currentState;
  static RobotState previousState;
  static bool stateEntered;      // true once one-shot entry actions have run
  static bool waitConfirmed;     // WAIT sub-state: exploration confirmed via Button1

  static unsigned long lastBlinkMs;
  static bool blinkOn;

  // debouncing
  static bool button1Pressed();
  static bool button2Pressed();
  static bool lastRawB1, lastRawB2;
  static unsigned long lastB1ChangeMs, lastB2ChangeMs;
  static bool stableB1, stableB2;

  static void handleCalibration();
  static void handleExploration();
  static void handleWait();
  static void handleSpeedRun();

  static void runExplorationRoutine();  // blocking: full maze exploration (there & back)
  static void runSpeedRunRoutine();     // blocking: shortest-path run to goal

  // shared helper: rotate robot from its current Floodfill heading to `target`
  // and drive forward exactly one cell, keeping Floodfill pose in sync.
  static bool stepToward(Heading target, float speedMMS);

  static void setLEDs(bool led1, bool led2);
  static void blinkLED(uint8_t pin, uint16_t periodMs);
  static void transitionTo(RobotState newState);
};

#endif // FSM_H
