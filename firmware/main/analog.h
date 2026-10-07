#pragma once

// =============================================================================
//  analog.h - ADC1 access
// =============================================================================
//
//  ADC1 in one-shot mode reads two inputs: the line sensor multiplexer output
//  and the battery divider. The read functions are not thread-safe and are
//  meant for the control task (or a caller holding the robot lock).
// =============================================================================

#include <stdbool.h>
#include "esp_err.h"

esp_err_t analog_init(void);

// Raw 12-bit reading (0..4095) of the 74HC4067 Z output; -1 on error.
int analog_read_line_raw(void);

// Calibrated voltage on the battery divider pin in millivolts; -1 on error.
int analog_read_battery_pin_mv(void);

// Raw battery-channel reading; returns the driver status (diagnostics).
int analog_read_battery_raw(int *raw);
