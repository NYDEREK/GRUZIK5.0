#pragma once

// =============================================================================
//  line_follower.h - PD line-following controller
// =============================================================================

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

// Settings sent by the app. Speeds are in PWM units (MOTOR_PWM_MAX = 100 %).
typedef struct {
    float kp;
    float kd;
    float base_speed;       // speed on a straight line
    float max_speed;        // upper limit for one wheel
    float sharp_right;      // search speeds right after the line is lost...
    float sharp_left;
    float bend_right;       // ...and after 300 ms without the line
    float bend_left;
    uint16_t threshold;     // raw reading at or above which a sensor sees black
} line_params_t;

typedef struct {
    bool valid;             // false when the controller is stopped
    float left;             // wheel commands in PWM units
    float right;
} line_output_t;

typedef struct {
    float position;         // 1000 (right edge) .. 16000 (left edge), 8500 = centre
    float error;            // 8500 - position
    uint8_t active_count;   // sensors currently on the line
    uint8_t last_end;       // edge that saw the line last: 0 = left, 1 = right
} line_status_t;

line_params_t *line_follower_params(void);
void line_follower_start(void);
void line_follower_stop(void);
line_output_t line_follower_update(const uint16_t raw[LINE_SENSOR_COUNT], uint32_t now_ms);
const line_status_t *line_follower_status(void);
