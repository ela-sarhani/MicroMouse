// ============================================================================
//  MicromouseRobot.ino
// ============================================================================
//  This file intentionally left (almost) empty.
//
//  Arduino IDE requires a .ino file matching the sketch folder name to open
//  the project, but all real firmware code lives in modular .h/.cpp files
//  sitting next to this .ino (Calibration, Sensors, Motors, PIDController,
//  Floodfill, FSM, main). They all show up as separate tabs in the IDE, and
//  setup()/loop() are actually defined in main.cpp, not here.
//
//  Required libraries (install via Library Manager before compiling):
//    - "Adafruit VL53L0X" by Adafruit          (Time-of-Flight sensors)
//    - "Adafruit BusIO"   by Adafruit          (dependency of the above)
//    - "MPU6050_light"    by rfetick           (IMU / yaw)
//
//  Board:  ESP32 Dev Module (ESP32-WROOM-32), esp32 core 3.x
// ============================================================================
