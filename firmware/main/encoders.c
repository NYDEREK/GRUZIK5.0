// =============================================================================
//  encoders.c - quadrature decoding in the PCNT peripheral
// =============================================================================
//
//  Each wheel uses one PCNT unit with two channels, which together count
//  every edge of both A and B (x4 decoding) with direction. The counting is
//  done entirely in hardware: at full speed the encoders produce hundreds of
//  thousands of edges per second, far too many for a GPIO interrupt.
//
//  The PCNT counter is 16-bit. With accum_count enabled and watch points at
//  the limits, the driver folds each overflow into a 32-bit total, so
//  pcnt_unit_get_count() returns the full count.
//
//  The index pulse (once per motor revolution) is rare and uses a plain GPIO
//  interrupt.
// =============================================================================

#include "encoders.h"

#include "board.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_check.h"

static const char *TAG = "encoders";

#define PCNT_LIMIT          30000   // hardware range before folding into 32 bit
#define GLITCH_FILTER_NS    1000    // ignore pulses shorter than 1 us

typedef struct {
    pcnt_unit_handle_t unit;
    int direction;                  // +1/-1: makes forward motion positive
    int32_t last_count;
    volatile uint32_t index_pulses;
    encoder_t state;
} wheel_t;

static wheel_t s_left = {.direction = ENCODER_LEFT_DIRECTION};
static wheel_t s_right = {.direction = ENCODER_RIGHT_DIRECTION};

static void IRAM_ATTR index_isr(void *arg)
{
    ++((wheel_t *)arg)->index_pulses;
}

static esp_err_t setup_wheel(wheel_t *wheel, gpio_num_t pin_a, gpio_num_t pin_b,
                             gpio_num_t pin_index)
{
    const pcnt_unit_config_t unit_cfg = {
        .low_limit = -PCNT_LIMIT,
        .high_limit = PCNT_LIMIT,
        .flags.accum_count = true,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &wheel->unit), TAG, "unit");
    const pcnt_glitch_filter_config_t filter = {.max_glitch_ns = GLITCH_FILTER_NS};
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(wheel->unit, &filter), TAG, "filter");

    // Channel A counts edges of A and looks at B for the direction; channel B
    // does the opposite. Together they count all four edges of a cycle.
    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;
    const pcnt_chan_config_t a_cfg = {.edge_gpio_num = pin_a, .level_gpio_num = pin_b};
    const pcnt_chan_config_t b_cfg = {.edge_gpio_num = pin_b, .level_gpio_num = pin_a};
    ESP_RETURN_ON_ERROR(pcnt_new_channel(wheel->unit, &a_cfg, &chan_a), TAG, "chan a");
    ESP_RETURN_ON_ERROR(pcnt_new_channel(wheel->unit, &b_cfg, &chan_b), TAG, "chan b");
    pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // Watch points at the limits let accum_count extend the range to 32 bit.
    pcnt_unit_add_watch_point(wheel->unit, PCNT_LIMIT);
    pcnt_unit_add_watch_point(wheel->unit, -PCNT_LIMIT);
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(wheel->unit), TAG, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(wheel->unit), TAG, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(wheel->unit), TAG, "start");

    // Keep the inputs defined when no encoder is plugged in.
    gpio_set_pull_mode(pin_a, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(pin_b, GPIO_PULLUP_ONLY);

    const gpio_config_t index_cfg = {
        .pin_bit_mask = 1ULL << pin_index,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&index_cfg), TAG, "index gpio");
    return gpio_isr_handler_add(pin_index, index_isr, wheel);
}

esp_err_t encoders_init(void)
{
    // The ISR service may already be installed by another module.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    ESP_RETURN_ON_ERROR(setup_wheel(&s_left, PIN_ENC_L_A, PIN_ENC_L_B, PIN_ENC_L_I), TAG, "left");
    ESP_RETURN_ON_ERROR(setup_wheel(&s_right, PIN_ENC_R_A, PIN_ENC_R_B, PIN_ENC_R_I), TAG, "right");
    return ESP_OK;
}

static void reset_wheel(wheel_t *wheel)
{
    pcnt_unit_clear_count(wheel->unit);
    wheel->last_count = 0;
    wheel->index_pulses = 0;
    wheel->state = (encoder_t){0};
}

void encoders_reset(void)
{
    reset_wheel(&s_left);
    reset_wheel(&s_right);
}

static void update_wheel(wheel_t *wheel, float dt_s)
{
    int count = 0;
    pcnt_unit_get_count(wheel->unit, &count);
    encoder_t *s = &wheel->state;
    s->count = count;
    s->delta = (count - wheel->last_count) * wheel->direction;
    wheel->last_count = count;
    s->index_pulses = wheel->index_pulses;

    const float revolutions = (float)s->delta / ENCODER_COUNTS_PER_WHEEL_REV;
    s->delta_m = revolutions * WHEEL_CIRCUMFERENCE_M;
    // In 1 ms a wheel moves only a few counts, so the raw speed jumps in
    // large steps; a low-pass filter smooths it.
    s->rpm += ENCODER_SPEED_FILTER_ALPHA * (revolutions / (dt_s / 60.0f) - s->rpm);
    s->mps += ENCODER_SPEED_FILTER_ALPHA * (s->delta_m / dt_s - s->mps);
}

void encoders_update(float dt_s)
{
    update_wheel(&s_left, dt_s);
    update_wheel(&s_right, dt_s);
}

const encoder_t *encoders_left(void)
{
    return &s_left.state;
}

const encoder_t *encoders_right(void)
{
    return &s_right.state;
}
