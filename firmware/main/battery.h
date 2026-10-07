#pragma once

// =============================================================================
//  battery.h - 2S LiPo monitor on the VIN divider
// =============================================================================

#include <stdbool.h>

// Takes one sample; call from the control task every BATTERY_PERIOD_MS.
void battery_update(void);

float battery_voltage(void);
float battery_percent(void);        // 0 % at BATTERY_EMPTY_V, 100 % at BATTERY_FULL_V
bool battery_present(void);         // false when running on USB power only

// PWM multiplier compensating a sagging pack, latched at each start.
// 1.0 at full charge, rising as the voltage drops; 1.0 without a pack.
float battery_speed_level(void);
