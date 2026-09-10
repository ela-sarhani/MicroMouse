#include "Calibration.h"

// ----------------------------------------------------------------------------
// DEFAULT CALIBRATION TABLES
// ----------------------------------------------------------------------------
// These start as an IDENTITY mapping (raw == actual), i.e. "no correction".
// Replace the "actual" column with your own measured ground-truth values once
// you've gathered raw readings from the Serial Monitor during STATE_CALIBRATION.
// Points MUST be sorted by ascending "raw" value.
// ----------------------------------------------------------------------------

const CalPoint Calibration::tableFL[] = {
 {31, 15}, {37, 30}, {62, 60}, {111, 100}, {161, 150}, {273, 250}, {432, 400}
};
const CalPoint Calibration::tableFR[] = {
 {45, 15}, {60, 30}, {87, 60}, {138, 100}, {184, 150}, {292, 250}, {449, 400}
};
const CalPoint Calibration::tableL[] = {
 {44, 15}, {54, 30}, {86, 60}, {127, 100}, {178, 150}, {300, 250}, {451, 400}
};
const CalPoint Calibration::tableR[] = {
 {26, 15}, {43, 30}, {76, 60}, {118, 100}, {171, 150}, {284, 250}, {438, 400}
};

const uint8_t Calibration::tableSizeFL = sizeof(tableFL) / sizeof(CalPoint);
const uint8_t Calibration::tableSizeFR = sizeof(tableFR) / sizeof(CalPoint);
const uint8_t Calibration::tableSizeL  = sizeof(tableL)  / sizeof(CalPoint);
const uint8_t Calibration::tableSizeR  = sizeof(tableR)  / sizeof(CalPoint);

// ----------------------------------------------------------------------------

uint16_t Calibration::interpolate(const CalPoint* table, uint8_t size, uint16_t raw) {
  if (size == 0) return raw;

  // Clamp below first point / above last point (linear extrapolation using
  // the nearest segment slope keeps behaviour sane outside the table range).
  if (raw <= table[0].raw) {
    if (size == 1) return table[0].actual;
    float slope = (float)(table[1].actual - table[0].actual) / (float)(table[1].raw - table[0].raw);
    return (uint16_t)constrain(table[0].actual + slope * (raw - table[0].raw), 0, TOF_MAX_RANGE_MM);
  }
  if (raw >= table[size - 1].raw) {
    if (size == 1) return table[size - 1].actual;
    float slope = (float)(table[size - 1].actual - table[size - 2].actual) /
                  (float)(table[size - 1].raw - table[size - 2].raw);
    return (uint16_t)constrain(table[size - 1].actual + slope * (raw - table[size - 1].raw), 0, TOF_MAX_RANGE_MM);
  }

  // Find bracketing segment and interpolate linearly within it.
  for (uint8_t i = 0; i < size - 1; i++) {
    if (raw >= table[i].raw && raw <= table[i + 1].raw) {
      uint16_t rawSpan = table[i + 1].raw - table[i].raw;
      if (rawSpan == 0) return table[i].actual;
      float t = (float)(raw - table[i].raw) / (float)rawSpan;
      return (uint16_t)(table[i].actual + t * (table[i + 1].actual - table[i].actual));
    }
  }
  return raw; // fallback, should not reach here
}

uint16_t Calibration::applyCalibration(uint8_t sensorIndex, uint16_t rawMM) {
  switch (sensorIndex) {
    case SENSOR_FL: return interpolate(tableFL, tableSizeFL, rawMM);
    case SENSOR_FR: return interpolate(tableFR, tableSizeFR, rawMM);
    case SENSOR_L:  return interpolate(tableL,  tableSizeL,  rawMM);
    case SENSOR_R:  return interpolate(tableR,  tableSizeR,  rawMM);
    default:        return rawMM;
  }
}

void Calibration::printCalibrationTable() {
  Serial.println(F("===== ToF Calibration Tables (raw_mm -> actual_mm) ====="));

  const CalPoint* tables[SENSOR_COUNT] = { tableFL, tableFR, tableL, tableR };
  const uint8_t sizes[SENSOR_COUNT]    = { tableSizeFL, tableSizeFR, tableSizeL, tableSizeR };
  const char* names[SENSOR_COUNT]      = { "FL", "FR", "L", "R" };

  for (uint8_t s = 0; s < SENSOR_COUNT; s++) {
    Serial.print(F("Sensor "));
    Serial.print(names[s]);
    Serial.print(F(": "));
    for (uint8_t i = 0; i < sizes[s]; i++) {
      Serial.print(tables[s][i].raw);
      Serial.print(F("->"));
      Serial.print(tables[s][i].actual);
      if (i < sizes[s] - 1) Serial.print(F(", "));
    }
    Serial.println();
  }
  Serial.println(F("=========================================================="));
}
