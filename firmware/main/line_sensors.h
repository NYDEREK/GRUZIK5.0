#pragma once

// =============================================================================
//  line_sensors.h - 16 reflective sensors behind a 74HC4067 multiplexer
// =============================================================================

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

void line_sensors_init(void);

// Reads the next LINE_CHANNELS_PER_TICK channels into raw[]. Values are stored
// in logical SENSOR0..SENSOR15 order, matching the numbering printed on the
// sensor board. Returns true when this call completed a full sweep.
// Control task only.
bool line_sensors_scan_step(uint16_t raw[LINE_SENSOR_COUNT]);
