#include "FSM.h"
#include "Sensors.h"
#include "Motors.h"
#include "PIDController.h"
#include "Floodfill.h"
#include "Calibration.h"

RobotState FSM::currentState = STATE_CALIBRATION;
RobotState FSM::previousState = STATE_CALIBRATION;
bool FSM::stateEntered = false;
bool FSM::waitConfirmed = false;

unsigned long FSM::lastBlinkMs = 0;
bool FSM::blinkOn = false;

bool FSM::lastRawB1 = HIGH;
bool FSM::lastRawB2 = HIGH;
unsigned long FSM::lastB1ChangeMs = 0;
unsigned long FSM::lastB2ChangeMs = 0;
bool FSM::stableB1 = HIGH;
bool FSM::stableB2 = HIGH;

// -----------------------------------------------------------------------------

void FSM::init() {
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  setLEDs(false, false);

  currentState = STATE_CALIBRATION;
  previousState = STATE_CALIBRATION;
  stateEntered = false;
  waitConfirmed = false;
}

RobotState FSM::getState() { return currentState; }

void FSM::transitionTo(RobotState newState) {
#if DEBUG_PRINT_FSM
  Serial.print(F("[FSM] transition -> "));
  Serial.println(newState);
#endif
  previousState = currentState;
  currentState = newState;
  stateEntered = false;
}

// -----------------------------------------------------------------------------
//  Debounced button reads (active LOW, INPUT_PULLUP). Returns true exactly
//  once on the press edge (HIGH->LOW), after debounce settling.
// -----------------------------------------------------------------------------
bool FSM::button1Pressed() {
  bool raw = digitalRead(PIN_BUTTON_1);
  if (raw != lastRawB1) {
    lastB1ChangeMs = millis();
    lastRawB1 = raw;
  }
  if ((millis() - lastB1ChangeMs) > BUTTON_DEBOUNCE_MS && raw != stableB1) {
    stableB1 = raw;
    if (stableB1 == LOW) return true; // press edge
  }
  return false;
}

bool FSM::button2Pressed() {
  bool raw = digitalRead(PIN_BUTTON_2);
  if (raw != lastRawB2) {
    lastB2ChangeMs = millis();
    lastRawB2 = raw;
  }
  if ((millis() - lastB2ChangeMs) > BUTTON_DEBOUNCE_MS && raw != stableB2) {
    stableB2 = raw;
    if (stableB2 == LOW) return true;
  }
  return false;
}

// -----------------------------------------------------------------------------

void FSM::setLEDs(bool led1, bool led2) {
  digitalWrite(PIN_LED_1, led1 ? HIGH : LOW);
  digitalWrite(PIN_LED_2, led2 ? HIGH : LOW);
}

void FSM::blinkLED(uint8_t pin, uint16_t periodMs) {
  if (millis() - lastBlinkMs >= periodMs) {
    lastBlinkMs = millis();
    blinkOn = !blinkOn;
  }
  digitalWrite(pin, blinkOn ? HIGH : LOW);
}

// -----------------------------------------------------------------------------

void FSM::update() {
  switch (currentState) {
    case STATE_CALIBRATION: handleCalibration(); break;
    case STATE_EXPLORATION: handleExploration(); break;
    case STATE_WAIT:        handleWait();        break;
    case STATE_SPEED_RUN:   handleSpeedRun();     break;
  }
}

// -----------------------------------------------------------------------------
//  STATE_CALIBRATION
// -----------------------------------------------------------------------------
void FSM::handleCalibration() {
  if (!stateEntered) {
    stateEntered = true;
    Serial.println(F("[FSM] STATE_CALIBRATION - keep robot stationary"));
    Sensors::calibrateGyro();
#if DEBUG_PRINT_CALIBRATION
    Calibration::printCalibrationTable();
#endif
    Serial.println(F("[FSM] Ready. Press Button 1 to start exploration."));
  }

  // Both LEDs slow-blink in unison to signal "ready / idle".
  if (millis() - lastBlinkMs >= LED_BLINK_SLOW_MS) {
    lastBlinkMs = millis();
    blinkOn = !blinkOn;
  }
  setLEDs(blinkOn, blinkOn);

  if (button1Pressed()) {
    transitionTo(STATE_EXPLORATION);
  }
}

// -----------------------------------------------------------------------------
//  STATE_EXPLORATION
// -----------------------------------------------------------------------------
void FSM::handleExploration() {
  if (!stateEntered) {
    stateEntered = true;
    setLEDs(true, false); // LED1 solid ON
    Serial.println(F("[FSM] STATE_EXPLORATION - mapping maze..."));
    runExplorationRoutine(); // BLOCKING: explores start->goal->start
    Serial.println(F("[FSM] Exploration complete, back at start."));
    waitConfirmed = false;
    transitionTo(STATE_WAIT);
  }
}

// One cell step: sense walls, update floodfill map, recompute flood, turn to
// face the best neighbor, drive forward one cell. Returns false if stuck.
bool FSM::stepToward(Heading target, float speedMMS) {
  int delta = ((int)target - (int)Floodfill::getCurrentHeading()) * 90;
  while (delta > 180) delta -= 360;
  while (delta < -180) delta += 360;

  if (delta != 0) {
    if (!Motion::turnInPlace((float)delta)) return false;
    Floodfill::setCurrentHeading(target);
  }

  if (!Motion::moveForward(MAZE_CELL_SIZE_MM, speedMMS)) return false;

  uint8_t nx = Floodfill::getCurrentX();
  uint8_t ny = Floodfill::getCurrentY();
  static const int8_t dx[4] = { 0, 1, 0, -1 };
  static const int8_t dy[4] = { 1, 0, -1, 0 };
  nx += dx[target];
  ny += dy[target];
  Floodfill::setCurrentPosition(nx, ny);
  return true;
}

void FSM::runExplorationRoutine() {
  Floodfill::init();

  const uint16_t MAX_STEPS = MAZE_SIZE * MAZE_SIZE * 4;
  uint16_t steps = 0;

  // ---- Phase 1: navigate Start -> Center goal, mapping walls as we go ----
  Floodfill::floodFillToGoal();
  while (!Floodfill::atGoal() && steps < MAX_STEPS) {
    Sensors::update();
    bool front = Sensors::wallFront();
    bool left  = Sensors::wallLeft();
    bool right = Sensors::wallRight();
    Floodfill::updateWallsFromLocalSense(front, left, right);
    Floodfill::floodFillToGoal();

    Heading next = Floodfill::getNextHeading();
    if (!Floodfill::canMove(next)) {
      Serial.println(F("[FSM] Exploration STUCK (goal phase)."));
      break;
    }
    if (!stepToward(next, SPEED_EXPLORE_MMS)) {
      Serial.println(F("[FSM] Motion primitive failed/timeout (goal phase)."));
      break;
    }
    steps++;
  }

  // ---- Phase 2: navigate Center -> Start, continuing to map walls ----
  Floodfill::floodFillToStart();
  while (!Floodfill::atStart() && steps < MAX_STEPS) {
    Sensors::update();
    bool front = Sensors::wallFront();
    bool left  = Sensors::wallLeft();
    bool right = Sensors::wallRight();
    Floodfill::updateWallsFromLocalSense(front, left, right);
    Floodfill::floodFillToStart();

    Heading next = Floodfill::getNextHeading();
    if (!Floodfill::canMove(next)) {
      Serial.println(F("[FSM] Exploration STUCK (return phase)."));
      break;
    }
    if (!stepToward(next, SPEED_EXPLORE_MMS)) {
      Serial.println(F("[FSM] Motion primitive failed/timeout (return phase)."));
      break;
    }
    steps++;
  }

  Motion::stopMotion();
}

// -----------------------------------------------------------------------------
//  STATE_WAIT
// -----------------------------------------------------------------------------
void FSM::handleWait() {
  if (!stateEntered) {
    stateEntered = true;
    Serial.println(F("[FSM] STATE_WAIT"));
  }

  if (!waitConfirmed) {
    // Awaiting confirmation that exploration is finalized: LED1 fast-blinks.
    blinkLED(PIN_LED_1, LED_BLINK_FAST_MS);
    digitalWrite(PIN_LED_2, LOW);

    if (button1Pressed()) {
      waitConfirmed = true;
      Serial.println(F("[FSM] Exploration finalized. Press Button 2 for speed run."));
      setLEDs(false, false);
      lastBlinkMs = millis();
    }
  } else {
    // Awaiting speed-run trigger: LED2 slow-blinks.
    digitalWrite(PIN_LED_1, LOW);
    blinkLED(PIN_LED_2, LED_BLINK_SLOW_MS);

    if (button2Pressed()) {
      transitionTo(STATE_SPEED_RUN);
    }
  }
}

// -----------------------------------------------------------------------------
//  STATE_SPEED_RUN
// -----------------------------------------------------------------------------
void FSM::handleSpeedRun() {
  if (!stateEntered) {
    stateEntered = true;
    setLEDs(false, true); // LED2 solid ON
    Serial.println(F("[FSM] STATE_SPEED_RUN - executing shortest path"));
    runSpeedRunRoutine();
    Serial.println(F("[FSM] Speed run complete."));
    waitConfirmed = true; // allow re-trigger of another speed run from WAIT
    transitionTo(STATE_WAIT);
  }
}

void FSM::runSpeedRunRoutine() {
  Floodfill::setCurrentPosition(MAZE_START_X, MAZE_START_Y);
  Floodfill::setCurrentHeading(HEADING_NORTH);
  Floodfill::computeShortestPath();

  for (uint16_t i = 0; i < Floodfill::shortestPathLength; i++) {
    Heading target = (Heading)Floodfill::shortestPathHeadings[i];
    if (!stepToward(target, SPEED_SPEEDRUN_MMS)) {
      Serial.println(F("[FSM] Speed run motion failure - aborting."));
      break;
    }
  }
  Motion::stopMotion();
}
