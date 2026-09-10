#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
//  GLOBAL DEBUG SWITCHES
// ============================================================================
#define DEBUG_SERIAL_BAUD          115200
#define DEBUG_PRINT_CALIBRATION    0   // print ToF calibration table + live raw/cal values
#define DEBUG_PRINT_SENSORS        0   // continuously print live sensor readings
#define DEBUG_PRINT_FSM            1   // print state transitions
#define DEBUG_PRINT_FLOODFILL      0   // print maze/distance grid after each flood

// ============================================================================
//  I2C BUS  (shared by MPU6050 + 4x VL53L0X)
// ============================================================================
#define PIN_I2C_SDA                21
#define PIN_I2C_SCL                22
#define I2C_CLOCK_HZ               400000UL

// ============================================================================
//  TIME-OF-FLIGHT SENSORS (VL53L0X) - XSHUT pins used for I2C address remap
// ============================================================================
#define PIN_XSHUT_FL                2   // Front-Left
#define PIN_XSHUT_FR                0   // Front-Right
#define PIN_XSHUT_L                 15   // Left
#define PIN_XSHUT_R                 4   // Right

// I2C addresses assigned to each sensor at boot (must all differ from 0x29 default)
#define TOF_ADDR_FL                 0x30
#define TOF_ADDR_FR                 0x31
#define TOF_ADDR_L                  0x32
#define TOF_ADDR_R                  0x33

// Sensor index ordering used throughout the codebase
enum ToFSensorIndex : uint8_t {
  SENSOR_FL = 0,
  SENSOR_FR = 1,
  SENSOR_L  = 2,
  SENSOR_R  = 3,
  SENSOR_COUNT = 4
};

#define TOF_FRONT_SETBACK_MM   5.78f   // e.g. 8
#define TOF_SIDE_SETBACK_MM    1.5f    // in mm

// Wall-detection thresholds (mm) - TUNE per maze wall material / sensor mount
#define WALL_THRESHOLD_FRONT_MM     110
#define WALL_THRESHOLD_SIDE_MM      90
#define TOF_MAX_RANGE_MM            2000
#define TOF_MEDIAN_SAMPLES          3    // must be odd

// ============================================================================
//  MOTOR DRIVER (TB6612FNG dual H-bridge)
// ============================================================================
#define PIN_MOTOR_L_AIN1            17
#define PIN_MOTOR_L_AIN2            16
#define PIN_MOTOR_L_PWM             19

#define PIN_MOTOR_R_BIN1            25
#define PIN_MOTOR_R_BIN2            26
#define PIN_MOTOR_R_PWM             14

#define PIN_MOTOR_STBY              27

#define PWM_FREQ_HZ                 20000
#define PWM_RESOLUTION_BITS         8
#define PWM_MAX_DUTY                255

// ============================================================================
//  QUADRATURE ENCODERS (input-only capable pins)
// ============================================================================
#define PIN_ENCODER_L_A              34
#define PIN_ENCODER_L_B              35
#define PIN_ENCODER_R_A              32
#define PIN_ENCODER_R_B              33

// ============================================================================
//  IMU (MPU6050) - Yaw only (gyro-Z integration, no magnetometer)
// ============================================================================
#define MPU_I2C_ADDR                 0x68
#define GYRO_CALIBRATION_SAMPLES     1500

// ============================================================================
//  USER INTERFACE
// ============================================================================
#define PIN_BUTTON_1                 18   // Start exploration / confirm
#define PIN_BUTTON_2                 23   // Start speed run
#define PIN_LED_1                    5    // Exploration indicator
#define PIN_LED_2                    13   // Speed run indicator

#define BUTTON_DEBOUNCE_MS           40
#define LED_BLINK_SLOW_MS            600
#define LED_BLINK_FAST_MS            150

// ============================================================================
//  PHYSICAL / MECHANICAL CONSTANTS  -- TUNE FOR YOUR ROBOT --
// ============================================================================
#define WHEEL_DIAMETER_MM            45.0f
#define WHEEL_BASE_MM                99.5f   // distance between L/R wheel contact points
#define ENCODER_CPR                  7.0f    // counts per motor-shaft revolution (encoder disc)
#define GEAR_RATIO                   150.0f   // N20 gearbox ratio (e.g. 1:150)
#define ENCODER_COUNTS_PER_WHEEL_REV (ENCODER_CPR * GEAR_RATIO * 4.0f) // x4 quadrature decode
#define MM_PER_ENCODER_COUNT         ((PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_WHEEL_REV)

#define MAZE_CELL_SIZE_MM            180.0f

// ============================================================================
//  MOTION TUNING
// ============================================================================
// Velocity control loop (per-wheel, PWM output from mm/s error)
#define VEL_PID_KP                   1.85f
#define VEL_PID_KI                   30.8f
#define VEL_PID_KD                   0.0f
#define VEL_PID_OUT_MIN              -255.0f
#define VEL_PID_OUT_MAX               255.0f

// Heading-hold PID (deg error -> differential wheel speed mm/s)
#define HEADING_PID_KP                6.0f
#define HEADING_PID_KI                0.05f
#define HEADING_PID_KD                0.4f
#define HEADING_PID_OUT_MIN          -250.0f
#define HEADING_PID_OUT_MAX           250.0f

// In-place turn PID (deg error -> wheel speed mm/s, opposite signs)
#define TURN_PID_KP                   4.5f
#define TURN_PID_KI                   0.02f
#define TURN_PID_KD                   0.25f
#define TURN_PID_OUT_MIN             -220.0f
#define TURN_PID_OUT_MAX              220.0f
#define TURN_DONE_TOLERANCE_DEG       1.5f
#define TURN_DONE_STABLE_MS           80

// Wall-centering PID (mm side error -> heading correction deg)
#define WALL_CENTER_KP                0.06f
#define WALL_CENTER_KI                0.0f
#define WALL_CENTER_KD                0.01f
#define WALL_CENTER_OUT_MIN          -20.0f
#define WALL_CENTER_OUT_MAX           20.0f

// Speeds (mm/s)
#define SPEED_EXPLORE_MMS             150.0f
#define SPEED_SPEEDRUN_MMS            600.0f
#define SPEED_TURN_MMS                180.0f
// Maximum achievable linear speed (empirically measured at full PWM under
// typical load). Set after running a calibration step; prevents commanding
// unattainable targets (e.g. 200 mm/s on this robot when max ~155-170 mm/s).
#define MAX_MOTOR_MMS                 155.0f

#define MOTION_LOOP_DT_MS             5     // control loop period (ms)
#define CELL_MOVE_TIMEOUT_MS           3000
#define TURN_TIMEOUT_MS                1500

// ============================================================================
//  MAZE / FLOODFILL
// ============================================================================
#define MAZE_SIZE                     16
#define MAZE_START_X                  0
#define MAZE_START_Y                  0
#define MAZE_GOAL_X_MIN                (MAZE_SIZE / 2 - 1)
#define MAZE_GOAL_X_MAX                (MAZE_SIZE / 2)
#define MAZE_GOAL_Y_MIN                (MAZE_SIZE / 2 - 1)
#define MAZE_GOAL_Y_MAX                (MAZE_SIZE / 2)

// Wall bitmask directions (absolute / global frame)
#define WALL_NORTH                    0x01
#define WALL_EAST                     0x02
#define WALL_SOUTH                    0x04
#define WALL_WEST                     0x08

enum Heading : uint8_t { HEADING_NORTH = 0, HEADING_EAST = 1, HEADING_SOUTH = 2, HEADING_WEST = 3 };

#endif // CONFIG_H
