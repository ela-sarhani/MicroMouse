# Micromouse Firmware — ESP32-WROOM-32

Modular Arduino IDE firmware for a differential-drive micromouse robot using
4x VL53L0X ToF sensors, an MPU6050 IMU, quadrature encoders, and a 4-state
FSM (Calibration → Exploration → Wait → Speed Run) driving a Floodfill maze
solver.

## 1. Opening the project

1. Copy the whole `MicromouseRobot/` folder (as-is) into your Arduino
   sketchbook folder.
2. Open `MicromouseRobot.ino` in Arduino IDE. Every `.h`/`.cpp` file sits
   directly next to the `.ino`, so they all appear as separate tabs in the
   IDE — click any tab to view/edit that module directly.
3. Board: **ESP32 Dev Module** (esp32 Arduino core, written against core 3.x
   APIs:
   - `ledcAttach(pin, freq, resBits)` / `ledcWrite(pin, duty)` in `Motors.cpp`
     — on core 2.x, replace with `ledcSetup()` + `ledcAttachPin()` +
     `ledcWrite(channel, duty)`.
   - `timerBegin(freqHz)` / `timerAttachInterrupt(timer, fn)` /
     `timerAlarm(timer, value, autoreload, count)` in `main.cpp` — on core
     2.x, replace with the 4-arg `timerBegin(num, divider, countUp)` +
     `timerAttachInterrupt(timer, fn, edge)` + `timerAlarmWrite()` +
     `timerAlarmEnable()` sequence instead.

## 2. Required Libraries (Library Manager)

| Library                | Author    | Purpose                     |
|-------------------------|-----------|------------------------------|
| Adafruit VL53L0X        | Adafruit  | ToF distance sensors         |
| Adafruit BusIO          | Adafruit  | dependency of the above      |
| MPU6050_light            | rfetick   | IMU (yaw via gyro-Z integration) |

## 3. Wiring / Pinout (edit in `src/config.h` if yours differs)

| Function                | ESP32 GPIO |
|--------------------------|------------|
| I2C SDA / SCL             | 21 / 22    |
| ToF XSHUT: FL / FR / L / R| 13 / 12 / 14 / 27 |
| Motor L: AIN1 / AIN2 / PWM| 25 / 26 / 33 |
| Motor R: BIN1 / BIN2 / PWM| 32 / 17 / 16 |
| Motor driver STBY         | 4          |
| Encoder L: A / B          | 34 / 35    |
| Encoder R: A / B          | 36 / 39    |
| Button 1 (explore/confirm)| 18 (INPUT_PULLUP, active LOW) |
| Button 2 (speed run)      | 19 (INPUT_PULLUP, active LOW) |
| LED 1                     | 5          |
| LED 2                     | 23         |

All 4 VL53L0X sensors share the same I2C bus (SDA/SCL). Their XSHUT pins are
used only once at boot to bring each sensor up individually and reassign it
a unique I2C address (0x30–0x33) — after `Sensors::init()` runs, all four
are addressable simultaneously on the shared bus.

Encoder pins 34/35/36/39 are ESP32 **input-only** GPIOs — perfect for
encoders (no PWM/output needed) but make sure your encoder modules have
push-pull outputs (most optical/Hall quadrature encoders do); these pins
have no internal pull resistors on most ESP32 variants.

## 4. File Structure

```
MicromouseRobot/
├── MicromouseRobot.ino      (entry stub - each tab below holds the real code)
├── README.md
├── config.h                  All pins, physical constants, PID gains, thresholds
├── Calibration.h/.cpp         Piecewise-linear ToF calibration lookup tables
├── Sensors.h/.cpp             ToF + IMU drivers, median filter, calibrated reads
├── Motors.h/.cpp               PWM driver, quadrature ISRs, velocity calc
├── PIDController.h/.cpp         Generic PID + Motion primitives (move/turn)
├── Floodfill.h/.cpp              Maze grid, BFS flood fill, shortest-path extraction
├── FSM.h/.cpp                     4-state state machine + exploration/speed-run routines
└── main.cpp                        setup()/loop(), timer tick, wiring it all together
```

All files live flat in the sketch folder (not in a subfolder) specifically so
Arduino IDE shows each one as its own clickable tab.

## 5. Calibrating the ToF sensors (this is what prints values to Serial)

`DEBUG_PRINT_CALIBRATION` is `1` by default in `config.h`. With it enabled:

- On boot, `Calibration::printCalibrationTable()` prints the current
  raw→actual lookup tables for all 4 sensors.
- Every call to `Sensors::update()` (which runs continuously while the robot
  is in `STATE_CALIBRATION` / `STATE_WAIT`, and internally during motion)
  prints a line like:
  ```
  [CAL] raw(FL,FR,L,R)=118,121,95,340  cal(FL,FR,L,R)=118,121,95,340
  ```

**To calibrate:** place a flat wall at known distances (e.g. 30/60/100/150/
250/400 mm) from each sensor, note the printed `raw` values in the Serial
Monitor, then edit the tables in `Calibration.cpp` (`tableFL/FR/L/R`) so
`actual` matches your tape-measure ground truth at each `raw` reading.
Re-flash — all reads are corrected automatically via piecewise-linear
interpolation. Set `DEBUG_PRINT_CALIBRATION 0` once you're done tuning to
reduce Serial spam and free up loop time.

## 6. Tuning checklist

All tunables live in `src/config.h`:

- **Mechanical**: `WHEEL_DIAMETER_MM`, `WHEEL_BASE_MM`, `ENCODER_CPR`,
  `GEAR_RATIO` — get `MM_PER_ENCODER_COUNT` right first, everything else
  (speed, PID) depends on it.
- **Wall thresholds**: `WALL_THRESHOLD_FRONT_MM`, `WALL_THRESHOLD_SIDE_MM`.
- **PID gains**: `VEL_PID_*` (per-wheel speed loop), `HEADING_PID_*`
  (straight-line heading hold), `TURN_PID_*` (in-place 90°/180° turns),
  `WALL_CENTER_*` (corridor centering using L/R ToF).
- **Speeds**: `SPEED_EXPLORE_MMS` (conservative) vs `SPEED_SPEEDRUN_MMS`
  (aggressive, tune last once explore-mode is reliable).

## 7. State machine summary

| State              | Trigger                | LED 1        | LED 2        |
|---------------------|-------------------------|--------------|--------------|
| CALIBRATION         | power-on                | slow blink   | slow blink   |
| EXPLORATION          | Button 1                | solid ON     | off          |
| WAIT (unconfirmed)   | robot back at start      | fast blink   | off          |
| WAIT (confirmed)     | Button 1 again           | off          | slow blink   |
| SPEED_RUN             | Button 2                | off          | solid ON     |

After a speed run finishes, the robot returns to WAIT (confirmed) so Button 2
can be pressed again to re-run the same shortest path.
