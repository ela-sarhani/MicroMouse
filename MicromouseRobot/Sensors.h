#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050_light.h>
#include "config.h"
#include "Calibration.h"

// Wraps the 4x VL53L0X ToF sensors (with XSHUT-based address remap) and the
// MPU6050 IMU (yaw only). Provides median-filtered, calibrated readings.
class Sensors {
public:
  static bool init();          // I2C bus + all sensor hardware bring-up
  static void update();        // poll all 4 ToF sensors, filter + calibrate
  static void updateIMU();     // integrate gyro-Z -> yaw (call at fixed dt)
  static void calibrateGyro(); // blocking: robot must be stationary

  static uint16_t getFL();
  static uint16_t getFR();
  static uint16_t getL();
  static uint16_t getR();

  static float getYaw();          // degrees, wrapped to [-180, 180)
  static void  resetYaw(float newYaw = 0.0f);

  static bool wallFront();
  static bool wallLeft();
  static bool wallRight();

  static void printDebug();

private:
  static Adafruit_VL53L0X tofFL;
  static Adafruit_VL53L0X tofFR;
  static Adafruit_VL53L0X tofL;
  static Adafruit_VL53L0X tofR;
  static MPU6050 mpu;

  static uint16_t distFL, distFR, distL, distR;

  static uint16_t histFL[TOF_MEDIAN_SAMPLES];
  static uint16_t histFR[TOF_MEDIAN_SAMPLES];
  static uint16_t histL[TOF_MEDIAN_SAMPLES];
  static uint16_t histR[TOF_MEDIAN_SAMPLES];
  static uint8_t  histIdx;

  static float yawOffset;
  static unsigned long lastImuMicros;

  static uint16_t medianOf3(uint16_t a, uint16_t b, uint16_t c);
  static bool bringUpSensor(Adafruit_VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddr, const char* name);
};

#endif // SENSORS_H
