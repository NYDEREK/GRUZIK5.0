#pragma once

// =============================================================================
//  odometry.h - robot pose from wheel encoders and the gyro
// =============================================================================
//
//  Coordinates: x forward at the start of the run, y to the left, yaw
//  counter-clockwise, all relative to the pose at the last reset.
// =============================================================================

#include <stdbool.h>

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;              // fused heading used for the pose
    float gyro_yaw_rad;         // heading from the gyro alone (diagnostics)
    float encoder_yaw_rad;      // heading from the encoders alone (diagnostics)
    float total_distance_m;     // path length driven
} odometry_t;

void odometry_reset(void);

// Integrates one step. gyro_valid = false falls back to encoder yaw only.
void odometry_update(float left_delta_m, float right_delta_m,
                     float gyro_delta_yaw_rad, bool gyro_valid);

const odometry_t *odometry_get(void);

// Wraps an angle into (-pi, pi].
float normalize_angle(float angle_rad);
