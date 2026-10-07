// =============================================================================
//  line_follower.c - PD line-following controller
// =============================================================================
//
//  Line position: every sensor reading at or above the threshold sees the
//  line. Each sensor has a weight (16000 on the left edge down to 1000 on the
//  right edge) and the position is the average weight of the active sensors.
//  The centre is 8500, so error = 8500 - position.
//
//  Control: correction = Kp * error + Kd * d(error); the left wheel gets
//  base + correction, the right wheel base - correction.
//
//  When the line disappears:
//    * on a straight section (line was centred, no edge sensor active) it is
//      probably a gap in the track: keep the last command for up to
//      LINE_GAP_BRIDGE_MS,
//    * otherwise the robot overshot a corner: turn towards the edge that saw
//      the line last, first with the "sharp" speeds, after 300 ms with the
//      softer "bend" speeds.
// =============================================================================

#include "line_follower.h"

#include <math.h>

#define CENTER_POSITION     8500.0f
#define SHARP_SEARCH_MS     300

static line_params_t s_params = {
    .kp = 0.015f,
    .kd = 0.55f,
    .base_speed = 125.0f,
    .max_speed = 200.0f,
    .sharp_right = -75.0f,
    .sharp_left = 120.0f,
    .bend_right = -75.0f,
    .bend_left = 120.0f,
    .threshold = 3300,
};

static struct {
    bool running;
    bool gap_armed;         // conditions for a gap were met on the last line contact
    bool gap_active;        // currently bridging a gap
    float last_error;
    float last_left;
    float last_right;
    uint32_t lost_since_ms; // 0 while the line is visible
    line_status_t status;
} s_lf = {.status = {.position = CENTER_POSITION}};

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

line_params_t *line_follower_params(void)
{
    return &s_params;
}

void line_follower_start(void)
{
    s_lf.running = true;
    s_lf.last_error = 0.0f;
    s_lf.lost_since_ms = 0;
    s_lf.gap_armed = false;
    s_lf.gap_active = false;
}

void line_follower_stop(void)
{
    s_lf.running = false;
    s_lf.gap_active = false;
    s_lf.gap_armed = false;
}

const line_status_t *line_follower_status(void)
{
    return &s_lf.status;
}

line_output_t line_follower_update(const uint16_t raw[LINE_SENSOR_COUNT], uint32_t now_ms)
{
    line_output_t out = {0};
    if (!s_lf.running) {
        return out;
    }

    // --- Where is the line? -------------------------------------------------------
    float weighted = 0.0f;
    uint8_t active = 0;
    bool edge_active = false;
    // The board is laid out SENSOR15 ... SENSOR0 from left to right.
    for (uint8_t physical = 0; physical < LINE_SENSOR_COUNT; ++physical) {
        if (raw[15 - physical] < s_params.threshold) {
            continue;
        }
        weighted += 16000.0f - physical * 1000.0f;
        ++active;
        if (physical <= 1) {
            s_lf.status.last_end = 0;
            edge_active = true;
        } else if (physical >= 14) {
            s_lf.status.last_end = 1;
            edge_active = true;
        }
    }
    s_lf.status.active_count = active;

    // --- Line visible: PD controller ------------------------------------------------
    if (active > 0) {
        s_lf.status.position = weighted / active;
        s_lf.status.error = CENTER_POSITION - s_lf.status.position;
        s_lf.lost_since_ms = 0;
        s_lf.gap_active = false;

        const float derivative = (s_lf.status.error - s_lf.last_error) * LINE_DERIVATIVE_SCALE;
        s_lf.last_error = s_lf.status.error;
        const float correction = s_params.kp * s_lf.status.error + s_params.kd * derivative;
        out.left = clampf(s_params.base_speed + correction, -MOTOR_PWM_MAX, s_params.max_speed);
        out.right = clampf(s_params.base_speed - correction, -MOTOR_PWM_MAX, s_params.max_speed);
        out.valid = true;
        s_lf.last_left = out.left;
        s_lf.last_right = out.right;
        // A narrow, centred line away from the edges: losing it now is a gap.
        s_lf.gap_armed = fabsf(s_lf.status.error) < 2000.0f && !edge_active && active <= 6;
        return out;
    }

    // --- Line lost ------------------------------------------------------------------
    if (s_lf.lost_since_ms == 0) {
        s_lf.lost_since_ms = now_ms;
    }
    const uint32_t lost_for = now_ms - s_lf.lost_since_ms;

    // Gap in the track: drive straight on with the last command.
    if ((s_lf.gap_active || s_lf.gap_armed) && lost_for <= LINE_GAP_BRIDGE_MS) {
        s_lf.gap_active = true;
        out.left = s_lf.last_left;
        out.right = s_lf.last_right;
        out.valid = true;
        return out;
    }

    // Overshot a corner: turn towards the edge that saw the line last.
    s_lf.gap_active = false;
    s_lf.gap_armed = false;
    const bool sharp = lost_for < SHARP_SEARCH_MS;
    const float outer = sharp ? s_params.sharp_left : s_params.bend_left;
    const float inner = sharp ? s_params.sharp_right : s_params.bend_right;
    if (s_lf.status.last_end == 1) {
        out.left = outer;
        out.right = inner;
    } else {
        out.left = inner;
        out.right = outer;
    }
    out.valid = true;
    return out;
}
