#pragma once

// =============================================================================
//  imu.h - yaw rate from one or two ICM-42688-P gyros
// =============================================================================
//
//  U7 sits on the top side of the board, the optional U10 on the bottom side
//  (upside down, so its yaw axis is reversed). With two sensors their yaw
//  rates are averaged. Enabled in menuconfig: GRUZIK5 -> Use the gyro(s).
// =============================================================================

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    IMU_MODE_NONE = 0,      // disabled or no sensor answered
    IMU_MODE_SINGLE = 1,
    IMU_MODE_DUAL = 2,
} imu_mode_t;

typedef struct {
    imu_mode_t mode;
    float gyro_z_raw_dps;   // as read, before bias removal
    float gyro_z_bias_dps;  // learned zero-rate offset
    float gyro_z_dps;       // corrected yaw rate, counter-clockwise positive
    float temperature_c;
} imu_state_t;

// Detects the sensors and calibrates the gyro bias. The robot must stand
// still for ~0.3 s during this call.
esp_err_t imu_init(void);

// Reads the gyro(s); call once per control tick. While the robot stands
// still, the bias slowly follows the reading to cancel thermal drift.
void imu_update(float dt_s, bool robot_stationary);

// Yaw angle accumulated since the previous call, in radians.
float imu_take_yaw_delta_rad(void);

const imu_state_t *imu_state(void);

// "IMU_MODE,..." status line for the app.
void imu_format_status(char *buf, size_t len);
