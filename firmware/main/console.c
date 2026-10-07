// =============================================================================
//  console.c - USB serial console
// =============================================================================

#include "console.h"

#include <stdio.h>
#include <string.h>
#include "analog.h"
#include "battery.h"
#include "board.h"
#include "commands.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_rom_sys.h"
#include "encoders.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "link.h"
#include "motors.h"
#include "robot.h"

#define STATUS_PERIOD_MS 2000

static void print_status(void)
{
    robot_lock();
    const robot_timing_t timing = robot_take_timing();
    const robot_mode_t mode = robot_mode();
    const encoder_t left = *encoders_left();
    const encoder_t right = *encoders_right();
    int battery_raw;
    const int battery_err = analog_read_battery_raw(&battery_raw);
    const int battery_mv = analog_read_battery_pin_mv();
    uint16_t raw[LINE_SENSOR_COUNT];
    for (int i = 0; i < LINE_SENSOR_COUNT; ++i) {
        raw[i] = robot_line_raw()[i];
    }
    robot_unlock();
    motor_status_t ml, mr;
    motors_get_status(&ml, &mr);

    printf("GRUZIK5 battery=%.2fV client=%d mode=%d imu=%d tick_max=%luus overruns=%lu\n",
           battery_voltage(), link_connected(), mode, imu_state()->mode,
           (unsigned long)timing.tick_max_us, (unsigned long)timing.overruns);
    printf("BATTERY pin raw=%d err=%d pin=%dmV\n", battery_raw, battery_err, battery_mv);
    printf("MOTOR nSLEEP=%d DRVOFF=%d L(dir=%d duty=%u%%) R(dir=%d duty=%u%%) speed_level=%.2f\n",
           gpio_get_level(PIN_MOTOR_NSLEEP), gpio_get_level(PIN_MOTOR_DRVOFF),
           ml.direction, ml.duty_percent, mr.direction, mr.duty_percent, motors_speed_level());
    printf("ENC L=%ld R=%ld idxL=%lu idxR=%lu\n", (long)left.count, (long)right.count,
           (unsigned long)left.index_pulses, (unsigned long)right.index_pulses);
    printf("LINE");
    for (int i = 0; i < LINE_SENSOR_COUNT; ++i) {
        printf(" S%02d=%u", i, raw[i]);
    }
    printf("\n");
}

// "PinDiag": proves whether the battery pin problem is in the ESP32 or on the
// board. Dumps the pad configuration, then briefly drives the pin high and
// reads it back through the ADC (expected ~4095 if the ADC path works).
static void battery_pin_diag(void)
{
    gpio_dump_io_configuration(stdout, 1ULL << PIN_BATTERY_ADC);
    robot_lock();
    int before, during, after;
    analog_read_battery_raw(&before);
    gpio_set_direction(PIN_BATTERY_ADC, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(PIN_BATTERY_ADC, 1);
    esp_rom_delay_us(2000);
    analog_read_battery_raw(&during);
    gpio_set_level(PIN_BATTERY_ADC, 0);
    gpio_set_direction(PIN_BATTERY_ADC, GPIO_MODE_DISABLE);
    esp_rom_delay_us(5000);
    analog_read_battery_raw(&after);
    robot_unlock();
    printf("PIN_DIAG raw before=%d driven_high=%d after=%d\n", before, during, after);
}

static void console_task(void *arg)
{
    char line[256];
    size_t len = 0;
    TickType_t last_status = xTaskGetTickCount();
    for (;;) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(50)) == 1) {
            if (c == '\n') {
                line[len] = '\0';
                printf("> %s\n", line);
                if (strcmp(line, "PinDiag") == 0) {
                    battery_pin_diag();
                }
                commands_execute(line);
                len = 0;
            } else if (c != '\r' && len < sizeof(line) - 1) {
                line[len++] = (char)c;
            }
        }
        if (xTaskGetTickCount() - last_status >= pdMS_TO_TICKS(STATUS_PERIOD_MS)) {
            last_status = xTaskGetTickCount();
            if (usb_serial_jtag_is_connected()) {
                print_status();
            }
        }
    }
}

void console_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&cfg);
    xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 2, NULL, 0);
}
