// =============================================================================
//  line_sensors.c - multiplexed line sensor scan
// =============================================================================
//
//  All 16 sensors share one ADC input through the 74HC4067. For every sensor:
//  select the channel on S0..S3, wait for the output to settle, convert.
//  The scan is split over two control ticks (8 channels each) so a tick stays
//  well below 1 ms.
// =============================================================================

#include "line_sensors.h"

#include "analog.h"
#include "board.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

// The two multiplexer banks are swapped on the sensor board:
// Y0..Y7 = SENSOR8..SENSOR15 and Y8..Y15 = SENSOR0..SENSOR7.
static const uint8_t SENSOR_BY_MUX_CHANNEL[LINE_SENSOR_COUNT] = {
    8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7,
};

void line_sensors_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LINE_MUX_S0) | (1ULL << PIN_LINE_MUX_S1) |
                        (1ULL << PIN_LINE_MUX_S2) | (1ULL << PIN_LINE_MUX_S3) |
                        (1ULL << PIN_LINE_MUX_E),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_LINE_MUX_E, 0);  // enable is active low
}

static void select_channel(uint8_t channel)
{
    gpio_set_level(PIN_LINE_MUX_S0, channel & 0x01);
    gpio_set_level(PIN_LINE_MUX_S1, (channel >> 1) & 0x01);
    gpio_set_level(PIN_LINE_MUX_S2, (channel >> 2) & 0x01);
    gpio_set_level(PIN_LINE_MUX_S3, (channel >> 3) & 0x01);
}

bool line_sensors_scan_step(uint16_t raw[LINE_SENSOR_COUNT])
{
    static uint8_t next_channel;  // where the previous call stopped
    for (int n = 0; n < LINE_CHANNELS_PER_TICK; ++n) {
        const uint8_t channel = next_channel;
        next_channel = (next_channel + 1) % LINE_SENSOR_COUNT;
        select_channel(channel);
        ets_delay_us(LINE_MUX_SETTLE_US);
        const int value = analog_read_line_raw();
        if (value >= 0) {
            raw[SENSOR_BY_MUX_CHANNEL[channel]] = (uint16_t)value;
        }
    }
    return next_channel == 0;  // wrapped around: every sensor is fresh
}
