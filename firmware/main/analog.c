// =============================================================================
//  analog.c - ADC1 one-shot driver with calibration
// =============================================================================
//
//  Uses the esp_adc one-shot API. Both channels run at 12 dB attenuation
//  (0..~3.1 V input range) and 12-bit resolution.
//
//  * Line sensors stay raw: the line threshold and the app diagnostics are
//    expressed in raw ADC counts.
//  * The battery uses the eFuse curve-fitting calibration, so the voltage is
//    accurate to a few tens of millivolts without manual trimming.
// =============================================================================

#include "analog.h"

#include "board.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "analog";

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_line_channel;
static adc_channel_t s_battery_channel;
static adc_cali_handle_t s_battery_cali;   // NULL when the chip has no calibration

// Maps a GPIO to its ADC1 channel and configures it.
static esp_err_t config_channel(gpio_num_t gpio, adc_channel_t *channel)
{
    adc_unit_t unit;
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(gpio, &unit, channel), TAG,
                        "GPIO%d is not an ADC pin", gpio);
    ESP_RETURN_ON_FALSE(unit == ADC_UNIT_1, ESP_ERR_INVALID_ARG, TAG,
                        "GPIO%d is not on ADC1", gpio);
    const adc_oneshot_chan_cfg_t cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_oneshot_config_channel(s_adc, *channel, &cfg);
}

esp_err_t analog_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "ADC1 unit");
    ESP_RETURN_ON_ERROR(config_channel(PIN_LINE_MUX_Z, &s_line_channel), TAG, "line channel");
    ESP_RETURN_ON_ERROR(config_channel(PIN_BATTERY_ADC, &s_battery_channel), TAG, "battery channel");

    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = s_battery_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_battery_cali) != ESP_OK) {
        ESP_LOGW(TAG, "no eFuse calibration, battery voltage is approximate");
        s_battery_cali = NULL;
    }
    return ESP_OK;
}

int analog_read_line_raw(void)
{
    int raw;
    return adc_oneshot_read(s_adc, s_line_channel, &raw) == ESP_OK ? raw : -1;
}

int analog_read_battery_raw(int *raw)
{
    *raw = -1;
    return adc_oneshot_read(s_adc, s_battery_channel, raw);
}

int analog_read_battery_pin_mv(void)
{
    int raw;
    if (adc_oneshot_read(s_adc, s_battery_channel, &raw) != ESP_OK) {
        return -1;
    }
    if (s_battery_cali == NULL) {
        return raw * 3300 / 4095;  // uncalibrated linear estimate
    }
    int mv;
    return adc_cali_raw_to_voltage(s_battery_cali, raw, &mv) == ESP_OK ? mv : -1;
}
