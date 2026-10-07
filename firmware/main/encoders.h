#pragma once

// =============================================================================
//  encoders.h - wheel encoders (IEP3-128, quadrature)
// =============================================================================

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int32_t count;          // raw accumulated count since the last reset
    int32_t delta;          // counts in the last tick, forward = positive
    float delta_m;          // distance travelled in the last tick
    float rpm;              // wheel speed, low-pass filtered
    float mps;              // wheel speed in m/s, low-pass filtered
    uint32_t index_pulses;  // index (I) pulses since the last reset
} encoder_t;

esp_err_t encoders_init(void);
void encoders_reset(void);

// Samples both counters and updates speeds; call once per control tick with
// the time elapsed since the previous call.
void encoders_update(float dt_s);

const encoder_t *encoders_left(void);
const encoder_t *encoders_right(void);
