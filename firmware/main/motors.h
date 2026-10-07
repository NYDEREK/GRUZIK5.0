#pragma once

// =============================================================================
//  motors.h - two DRV8245H-Q1 H-bridges in PH/EN mode
// =============================================================================

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int8_t direction;       // -1 backward, 0 stopped, +1 forward
    uint8_t duty_percent;   // 0..100
} motor_status_t;

// Configures the outputs and runs the driver wake-up handshake once.
// The battery must already be connected: the drivers only accept the
// acknowledge pulse while their motor supply (VM) is present.
esp_err_t motors_init(void);

// Sets both wheels in PWM units (+-MOTOR_PWM_MAX), multiplied by the speed
// level. Positive = forward.
void motors_drive(float left_pwm, float right_pwm);

// Zero duty on both wheels. The drivers stay awake.
void motors_stop(void);

void motors_set_speed_level(float level);
float motors_speed_level(void);

void motors_get_status(motor_status_t *left, motor_status_t *right);
