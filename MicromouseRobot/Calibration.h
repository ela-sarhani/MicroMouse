#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "config.h"

// A single (raw_mm -> actual_mm) calibration anchor point.
struct CalPoint {
  uint16_t raw;
  uint16_t actual;
};

// Piecewise-linear interpolation engine + per-sensor lookup tables.
//
// HOW TO CALIBRATE:
//   1. Place a flat wall at known distances (e.g. 30, 60, 100, 150, 250, 400 mm)
//      from each sensor.
//   2. Set DEBUG_PRINT_CALIBRATION 1 in config.h and watch the Serial Monitor
//      while in STATE_CALIBRATION - raw readings will be printed for you.
//   3. Fill in the "actual" measured distance for each "raw" printed value in
//      the tables below (Calibration.cpp).
//   4. Re-flash. All sensor reads are auto-corrected via applyCalibration().
class Calibration {
public:
  // Applies piecewise-linear calibration for the given sensor index.
  static uint16_t applyCalibration(uint8_t sensorIndex, uint16_t rawMM);

  // Prints the full calibration table for every sensor to Serial.
  static void printCalibrationTable();

private:
  static uint16_t interpolate(const CalPoint* table, uint8_t size, uint16_t raw);

  static const CalPoint tableFL[];
  static const CalPoint tableFR[];
  static const CalPoint tableL[];
  static const CalPoint tableR[];

  static const uint8_t tableSizeFL;
  static const uint8_t tableSizeFR;
  static const uint8_t tableSizeL;
  static const uint8_t tableSizeR;
};

#endif // CALIBRATION_H
