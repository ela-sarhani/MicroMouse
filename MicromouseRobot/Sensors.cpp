#include "Sensors.h"

// ---- static member storage -------------------------------------------------
Adafruit_VL53L0X Sensors::tofFL;
Adafruit_VL53L0X Sensors::tofFR;
Adafruit_VL53L0X Sensors::tofL;
Adafruit_VL53L0X Sensors::tofR;
MPU6050 Sensors::mpu(Wire);

uint16_t Sensors::distFL = TOF_MAX_RANGE_MM;
uint16_t Sensors::distFR = TOF_MAX_RANGE_MM;
uint16_t Sensors::distL  = TOF_MAX_RANGE_MM;
uint16_t Sensors::distR  = TOF_MAX_RANGE_MM;

uint16_t Sensors::histFL[TOF_MEDIAN_SAMPLES] = {0};
uint16_t Sensors::histFR[TOF_MEDIAN_SAMPLES] = {0};
uint16_t Sensors::histL[TOF_MEDIAN_SAMPLES]  = {0};
uint16_t Sensors::histR[TOF_MEDIAN_SAMPLES]  = {0};
uint8_t  Sensors::histIdx = 0;

float Sensors::yawOffset = 0.0f;
unsigned long Sensors::lastImuMicros = 0;

// -----------------------------------------------------------------------------

bool Sensors::bringUpSensor(Adafruit_VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddr, const char* name) {
  digitalWrite(xshutPin, HIGH);
  delay(10);
  if (!sensor.begin(newAddr, false, &Wire)) {
    Serial.print(F("[Sensors] FAILED to init VL53L0X: "));
    Serial.println(name);
    return false;
  }
  Serial.print(F("[Sensors] VL53L0X "));
  Serial.print(name);
  Serial.print(F(" ready @ 0x"));
  Serial.println(newAddr, HEX);
  return true;
}

bool Sensors::init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  // --- ToF XSHUT address remap sequence ---
  pinMode(PIN_XSHUT_FL, OUTPUT);
  pinMode(PIN_XSHUT_FR, OUTPUT);
  pinMode(PIN_XSHUT_L,  OUTPUT);
  pinMode(PIN_XSHUT_R,  OUTPUT);

  // Hold all sensors in reset first.
  digitalWrite(PIN_XSHUT_FL, LOW);
  digitalWrite(PIN_XSHUT_FR, LOW);
  digitalWrite(PIN_XSHUT_L,  LOW);
  digitalWrite(PIN_XSHUT_R,  LOW);
  delay(20);

  bool ok = true;
  ok &= bringUpSensor(tofFL, PIN_XSHUT_FL, TOF_ADDR_FL, "FL");
  ok &= bringUpSensor(tofFR, PIN_XSHUT_FR, TOF_ADDR_FR, "FR");
  ok &= bringUpSensor(tofL,  PIN_XSHUT_L,  TOF_ADDR_L,  "L");
  ok &= bringUpSensor(tofR,  PIN_XSHUT_R,  TOF_ADDR_R,  "R");

  // --- MPU6050 ---
  byte mpuStatus = mpu.begin();
  if (mpuStatus != 0) {
    Serial.print(F("[Sensors] MPU6050 init FAILED, status="));
    Serial.println(mpuStatus);
    ok = false;
  } else {
    Serial.println(F("[Sensors] MPU6050 ready"));
  }

  lastImuMicros = micros();
  return ok;
}

void Sensors::calibrateGyro() {
  Serial.println(F("[Sensors] Calibrating gyro offsets - keep robot STILL..."));
  mpu.calcOffsets(true, true); // gyro + accel offsets, blocking
  yawOffset = 0.0f;
  Serial.println(F("[Sensors] Gyro calibration complete."));
}

uint16_t Sensors::medianOf3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

void Sensors::update() {
  VL53L0X_RangingMeasurementData_t m;

  tofFL.rangingTest(&m, false);
  uint16_t rawFL = (m.RangeStatus != 4) ? m.RangeMilliMeter : TOF_MAX_RANGE_MM;
  tofFR.rangingTest(&m, false);
  uint16_t rawFR = (m.RangeStatus != 4) ? m.RangeMilliMeter : TOF_MAX_RANGE_MM;
  tofL.rangingTest(&m, false);
  uint16_t rawL = (m.RangeStatus != 4) ? m.RangeMilliMeter : TOF_MAX_RANGE_MM;
  tofR.rangingTest(&m, false);
  uint16_t rawR = (m.RangeStatus != 4) ? m.RangeMilliMeter : TOF_MAX_RANGE_MM;

  rawFL = constrain(rawFL, 0, TOF_MAX_RANGE_MM);
  rawFR = constrain(rawFR, 0, TOF_MAX_RANGE_MM);
  rawL  = constrain(rawL,  0, TOF_MAX_RANGE_MM);
  rawR  = constrain(rawR,  0, TOF_MAX_RANGE_MM);

  // rolling 3-sample buffers -> median filter (rejects single-sample spikes)
  histFL[histIdx] = rawFL;
  histFR[histIdx] = rawFR;
  histL[histIdx]  = rawL;
  histR[histIdx]  = rawR;
  histIdx = (histIdx + 1) % TOF_MEDIAN_SAMPLES;

  uint16_t medFL = medianOf3(histFL[0], histFL[1], histFL[2]);
  uint16_t medFR = medianOf3(histFR[0], histFR[1], histFR[2]);
  uint16_t medL  = medianOf3(histL[0],  histL[1],  histL[2]);
  uint16_t medR  = medianOf3(histR[0],  histR[1],  histR[2]);

  distFL = Calibration::applyCalibration(SENSOR_FL, medFL);
  distFR = Calibration::applyCalibration(SENSOR_FR, medFR);
  distL  = Calibration::applyCalibration(SENSOR_L,  medL);
  distR  = Calibration::applyCalibration(SENSOR_R,  medR);

#if DEBUG_PRINT_CALIBRATION
  Serial.print(F("[CAL] raw(FL,FR,L,R)="));
  Serial.print(rawFL); Serial.print(',');
  Serial.print(rawFR); Serial.print(',');
  Serial.print(rawL);  Serial.print(',');
  Serial.print(rawR);
  Serial.print(F("  cal(FL,FR,L,R)="));
  Serial.print(distFL); Serial.print(',');
  Serial.print(distFR); Serial.print(',');
  Serial.print(distL);  Serial.print(',');
  Serial.println(distR);
#endif

#if DEBUG_PRINT_SENSORS
  printDebug();
#endif
}

void Sensors::updateIMU() {
  mpu.update();
}

float Sensors::getYaw() {
  float y = mpu.getAngleZ() - yawOffset;
  while (y >= 180.0f)  y -= 360.0f;
  while (y < -180.0f)  y += 360.0f;
  return y;
}

void Sensors::resetYaw(float newYaw) {
  yawOffset = mpu.getAngleZ() - newYaw;
}

uint16_t Sensors::getFL() { return distFL; }
uint16_t Sensors::getFR() { return distFR; }
uint16_t Sensors::getL()  { return distL; }
uint16_t Sensors::getR()  { return distR; }

bool Sensors::wallFront() {
  return (distFL < WALL_THRESHOLD_FRONT_MM) || (distFR < WALL_THRESHOLD_FRONT_MM);
}
bool Sensors::wallLeft()  { return distL < WALL_THRESHOLD_SIDE_MM; }
bool Sensors::wallRight() { return distR < WALL_THRESHOLD_SIDE_MM; }

void Sensors::printDebug() {
  Serial.print(F("FL:")); Serial.print(distFL);
  Serial.print(F(" FR:")); Serial.print(distFR);
  Serial.print(F(" L:"));  Serial.print(distL);
  Serial.print(F(" R:"));  Serial.print(distR);
  Serial.print(F(" Yaw:")); Serial.println(getYaw());
}