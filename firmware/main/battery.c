// =============================================================================
//  battery.c - battery voltage and speed level
// =============================================================================

#include "battery.h"

#include "analog.h"
#include "config.h"

// One conversion per update keeps the control tick short; a light low-pass
// filter removes noise. The very first sample is taken as is.
#define BATTERY_FILTER_ALPHA 0.3f

static volatile float s_voltage;
static bool s_has_sample;

void battery_update(void)
{
    const int mv = analog_read_battery_pin_mv();
    if (mv < 0) {
        return;
    }
    const float volts = mv / 1000.0f * BATTERY_DIVIDER;
    s_voltage = s_has_sample ? s_voltage + BATTERY_FILTER_ALPHA * (volts - s_voltage) : volts;
    s_has_sample = true;
}

float battery_voltage(void)
{
    return s_voltage;
}

bool battery_present(void)
{
    return s_voltage >= BATTERY_PRESENT_V;
}

float battery_percent(void)
{
    const float percent = 100.0f * (s_voltage - BATTERY_EMPTY_V) /
                          (BATTERY_FULL_V - BATTERY_EMPTY_V);
    return percent < 0.0f ? 0.0f : (percent > 100.0f ? 100.0f : percent);
}

float battery_speed_level(void)
{
    // On USB power the divider reads ~0 V; scaling would double the PWM.
    if (!battery_present()) {
        return 1.0f;
    }
    const float percent = s_voltage / SPEED_LEVEL_REFERENCE_V * 100.0f;
    const float level = (200.0f - percent) / 100.0f - SPEED_LEVEL_OFFSET;
    return level < 1.0f ? 1.0f : level;
}
