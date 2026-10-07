// =============================================================================
//  motors.c - DRV8245H-Q1 motor drivers
// =============================================================================
//
//  Each wheel has one DRV8245H in PH/EN mode:
//    EN/IN1  <- LEDC PWM, 20 kHz, 10 bit  (speed)
//    PH/IN2  <- GPIO                      (direction)
//  Both drivers share nSLEEP (wake/sleep) and DRVOFF (high = outputs Hi-Z).
//
//  The hardware variant of the DRV8245 refuses to drive after power-up or
//  wake-up until the controller acknowledges with a short reset pulse on
//  nSLEEP. That handshake runs once in motors_init(); afterwards the drivers
//  stay awake for the whole session and stopping only sets the duty to zero.
// =============================================================================

#include "motors.h"

#include <math.h>
#include "board.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static const char *TAG = "motors";

#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_CH_LEFT     LEDC_CHANNEL_0
#define LEDC_CH_RIGHT    LEDC_CHANNEL_1
#define DUTY_MAX         ((1u << MOTOR_PWM_RESOLUTION_BITS) - 1u)

// Pause between dropping the duty and flipping PH, so the bridge never
// reverses while it is driving current.
#define DIRECTION_CHANGE_US  150

typedef struct {
    gpio_num_t dir_pin;
    ledc_channel_t channel;
    motor_status_t status;
} motor_t;

static motor_t s_left = {.dir_pin = PIN_MOTOR_L_DIR, .channel = LEDC_CH_LEFT};
static motor_t s_right = {.dir_pin = PIN_MOTOR_R_DIR, .channel = LEDC_CH_RIGHT};
static float s_speed_level = 1.0f;

static void set_duty(ledc_channel_t channel, uint8_t percent)
{
    ledc_set_duty(LEDC_MODE, channel, DUTY_MAX * percent / 100u);
    ledc_update_duty(LEDC_MODE, channel);
}

// Wake-up handshake (DRV8245 datasheet, section 6.7.2.1):
//   1. nSLEEP low for 100 ms   - hard reset into sleep
//   2. nSLEEP high, wait 5 ms  - wake up, wait longer than tREADY (1 ms)
//   3. nSLEEP low for 15 us    - acknowledge pulse (inside the 5..20 us
//                                tRESET window, well below tSLEEP = 40 us,
//                                so the driver does not go back to sleep)
//   4. nSLEEP high, wait 5 ms  - driver settles in STANDBY
//   5. DRVOFF low              - enable the bridges
// Without step 3 nFAULT stays low and the outputs stay Hi-Z.
static void wake_drivers(void)
{
    gpio_set_level(PIN_MOTOR_NSLEEP, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_MOTOR_NSLEEP, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(PIN_MOTOR_NSLEEP, 0);
    ets_delay_us(15);
    gpio_set_level(PIN_MOTOR_NSLEEP, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(PIN_MOTOR_DRVOFF, 0);
}

esp_err_t motors_init(void)
{
    // Direction and control lines. INPUT_OUTPUT so the levels can be read
    // back in the diagnostics.
    const gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_L_DIR) | (1ULL << PIN_MOTOR_R_DIR) |
                        (1ULL << PIN_MOTOR_NSLEEP) | (1ULL << PIN_MOTOR_DRVOFF),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out_cfg), TAG, "gpio");
    gpio_set_level(PIN_MOTOR_L_DIR, 0);
    gpio_set_level(PIN_MOTOR_R_DIR, 0);
    gpio_set_level(PIN_MOTOR_DRVOFF, 1);   // outputs off until the handshake is done
    gpio_set_level(PIN_MOTOR_NSLEEP, 0);

    // One shared PWM timer, one channel per wheel, both starting at 0 %.
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION_BITS,
        .freq_hz = MOTOR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "ledc timer");
    const gpio_num_t pwm_pins[2] = {PIN_MOTOR_L_PWM, PIN_MOTOR_R_PWM};
    const ledc_channel_t channels[2] = {LEDC_CH_LEFT, LEDC_CH_RIGHT};
    for (int i = 0; i < 2; ++i) {
        const ledc_channel_config_t ch_cfg = {
            .gpio_num = pwm_pins[i],
            .speed_mode = LEDC_MODE,
            .channel = channels[i],
            .timer_sel = LEDC_TIMER,
            .duty = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "ledc channel");
    }

    wake_drivers();
    return ESP_OK;
}

// Applies one wheel command: PWM units -> direction + duty percent.
static void set_motor(motor_t *motor, float pwm)
{
    pwm *= s_speed_level;
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;
    const int8_t direction = pwm > 0.0f ? 1 : (pwm < 0.0f ? -1 : 0);
    const uint8_t duty = (uint8_t)lroundf(fabsf(pwm) / MOTOR_PWM_MAX * 100.0f);

    if (direction != 0 && direction != motor->status.direction) {
        set_duty(motor->channel, 0);
        ets_delay_us(DIRECTION_CHANGE_US);
    }
    if (direction != 0) {
        gpio_set_level(motor->dir_pin, direction > 0 ? 1 : 0);
    }
    motor->status.direction = direction;
    motor->status.duty_percent = duty;
    set_duty(motor->channel, duty);
}

void motors_drive(float left_pwm, float right_pwm)
{
    set_motor(&s_left, left_pwm);
    set_motor(&s_right, right_pwm);
}

void motors_stop(void)
{
    set_duty(LEDC_CH_LEFT, 0);
    set_duty(LEDC_CH_RIGHT, 0);
    s_left.status = (motor_status_t){0};
    s_right.status = (motor_status_t){0};
}

void motors_set_speed_level(float level)
{
    s_speed_level = level;
}

float motors_speed_level(void)
{
    return s_speed_level;
}

void motors_get_status(motor_status_t *left, motor_status_t *right)
{
    *left = s_left.status;
    *right = s_right.status;
}
