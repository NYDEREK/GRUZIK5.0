#pragma once

// =============================================================================
//  route_map.h - route recording and playback
// =============================================================================
//
//  Mapping: while following the line, the robot records its pose every 40 ms
//  to GRUZIK.txt. The app downloads it, optimises the speeds per point and
//  uploads the result as map.txt.
//
//  Playback: the robot drives map.txt on odometry - it steers towards the
//  route with a pure-pursuit style controller and adds a small correction
//  from the line sensors, so it can go faster than the plain line PID.
// =============================================================================

#include <stdbool.h>
#include <stdint.h>
#include "line_follower.h"
#include "odometry.h"

typedef struct {
    float p;
    float i;
    float d;
    float default_speed;
} map_params_t;

map_params_t *route_map_params(void);
void route_map_init(void);

// --- Recording -----------------------------------------------------------
bool route_map_begin_recording(const odometry_t *odom);
// Call every control tick while mapping. Returns true when the route closed
// into a loop near the start; the recording is then finished.
bool route_map_record_tick(const odometry_t *odom, float speed_mps,
                           float line_error, uint32_t now_ms);
void route_map_end_recording(void);
uint16_t route_map_recorded_points(void);

// --- Playback ------------------------------------------------------------
bool route_map_begin_playback(void);
// Computes the wheel commands for this tick; returns false once the last
// route point has been passed.
bool route_map_playback_step(const odometry_t *odom, const line_output_t *line,
                             const line_status_t *line_status,
                             float *left_pwm, float *right_pwm);

// --- Transfer to/from the app ----------------------------------------------
// Called from the command task while the robot is stopped.
void route_map_dump(bool optimized);
void route_map_upload_begin(int expected_points);
void route_map_upload_point(const char *value);
void route_map_upload_end(void);
void route_map_append_point(float x, float y);
