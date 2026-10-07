#pragma once

// =============================================================================
//  robot.h - robot modes and the 1 kHz control loop
// =============================================================================
//
//  Every function except robot_init() must be called with the robot state
//  locked (robot_lock / robot_unlock). The control loop takes the same lock
//  for each tick, so an app command can never land in the middle of the
//  computations.
// =============================================================================

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

// What the robot is doing right now.
typedef enum {
    ROBOT_STOPPED = 0,
    ROBOT_NORMAL,       // line following on PID
    ROBOT_MAPPING,      // PID + route recording
    ROBOT_PLAYBACK,     // driving a stored route
    ROBOT_MANUAL,       // joystick
    ROBOT_CLEANING,     // tyre cleaning
} robot_mode_t;

// Which mode the "Start" button in the app runs.
typedef enum {
    ROBOT_SELECT_NORMAL,
    ROBOT_SELECT_MAPPING,
    ROBOT_SELECT_PLAYBACK,
} robot_selection_t;

typedef enum {
    TELEMETRY_OFF,
    TELEMETRY_ODOM,     // pose, wheel speeds, battery
    TELEMETRY_DEBUG,    // + line sensors, encoders, gyro
} telemetry_mode_t;

// Tick timing statistics for the USB log.
typedef struct {
    uint32_t tick_max_us;   // longest tick since the previous read
    uint32_t overruns;      // late ticks since boot
} robot_timing_t;

// Creates the state lock and starts the control task on core 1.
void robot_init(void);
void robot_lock(void);
void robot_unlock(void);

robot_mode_t robot_mode(void);
void robot_select(robot_selection_t selection);
void robot_start_selected(void);
void robot_start_normal(void);
void robot_start_mapping(void);
void robot_start_playback(void);
void robot_stop(void);              // stops and reports "Stop" + battery voltage
void robot_manual(float left_pwm, float right_pwm);
void robot_clean(bool on);
void robot_set_clean_speed(float pwm);

void robot_set_telemetry(telemetry_mode_t mode);
telemetry_mode_t robot_telemetry(void);

// Latest raw readings of the 16 line sensors (SENSOR0..SENSOR15 order).
const uint16_t *robot_line_raw(void);

// Returns the timing statistics and resets the maximum.
robot_timing_t robot_take_timing(void);
