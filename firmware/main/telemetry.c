// =============================================================================
//  telemetry.c - ODOM / DBG lines
// =============================================================================
//
//  ODOM,x,y,yaw_deg,distance,v_left,v_right,gyro_dps,enc_yaw_deg,gyro_yaw_deg
//  DBG,position,active,last_end,delta_l,delta_r,rpm_l,rpm_r,dist_l,dist_r,
//      gyro_raw,gyro_bias,gyro,S00..S15
//
//  A snapshot of the robot state is copied under the lock; formatting and
//  sending happen afterwards, so the control loop is blocked only for the
//  copy.
// =============================================================================

#include "telemetry.h"

#include <math.h>
#include <stdio.h>
#include "battery.h"
#include "config.h"
#include "encoders.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "line_follower.h"
#include "link.h"
#include "odometry.h"
#include "robot.h"

#define RAD_TO_DEG (180.0f / (float)M_PI)

static float wheel_distance(const encoder_t *encoder, int direction)
{
    return (float)encoder->count / ENCODER_COUNTS_PER_WHEEL_REV * WHEEL_CIRCUMFERENCE_M * direction;
}

void telemetry_send_hello(void)
{
    char imu[96];
    imu_format_status(imu, sizeof(imu));
    char msg[192];
    snprintf(msg, sizeof(msg), "GRUZIK5,ready\r\n%sBattery = %.2f V\r\n", imu, battery_voltage());
    link_send(msg);
}

static void send_frame(telemetry_mode_t mode, bool with_battery)
{
    char odom_line[160];
    char debug_line[320];
    debug_line[0] = '\0';

    robot_lock();
    const odometry_t odom = *odometry_get();
    const encoder_t left = *encoders_left();
    const encoder_t right = *encoders_right();
    const imu_state_t imu = *imu_state();
    const line_status_t line = *line_follower_status();
    uint16_t raw[LINE_SENSOR_COUNT];
    for (int i = 0; i < LINE_SENSOR_COUNT; ++i) {
        raw[i] = robot_line_raw()[i];
    }
    robot_unlock();

    snprintf(odom_line, sizeof(odom_line), "ODOM,%.4f,%.4f,%.2f,%.4f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
             odom.x_m, odom.y_m, odom.yaw_rad * RAD_TO_DEG, odom.total_distance_m,
             left.mps, right.mps, imu.gyro_z_dps, odom.encoder_yaw_rad * RAD_TO_DEG,
             odom.gyro_yaw_rad * RAD_TO_DEG);
    link_send_quiet(odom_line);

    if (with_battery) {
        char msg[40];
        snprintf(msg, sizeof(msg), "Battery = %.2f V\r\n", battery_voltage());
        link_send_quiet(msg);
    }
    if (mode != TELEMETRY_DEBUG) {
        return;
    }
    int n = snprintf(debug_line, sizeof(debug_line),
                     "DBG,%.0f,%u,%u,%ld,%ld,%.1f,%.1f,%.4f,%.4f,%.2f,%.2f,%.2f",
                     line.position, line.active_count, line.last_end,
                     (long)left.delta, (long)right.delta, left.rpm, right.rpm,
                     wheel_distance(&left, ENCODER_LEFT_DIRECTION),
                     wheel_distance(&right, ENCODER_RIGHT_DIRECTION),
                     imu.gyro_z_raw_dps, imu.gyro_z_bias_dps, imu.gyro_z_dps);
    for (int i = 0; i < LINE_SENSOR_COUNT && n < (int)sizeof(debug_line) - 8; ++i) {
        n += snprintf(debug_line + n, sizeof(debug_line) - n, ",%u", raw[i]);
    }
    snprintf(debug_line + n, sizeof(debug_line) - n, "\r\n");
    link_send_quiet(debug_line);
}

static void telemetry_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint32_t since_battery_ms = 0;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        since_battery_ms += TELEMETRY_PERIOD_MS;
        robot_lock();
        const telemetry_mode_t mode = robot_telemetry();
        robot_unlock();
        if (mode == TELEMETRY_OFF || !link_connected()) {
            continue;
        }
        const bool with_battery = since_battery_ms >= TELEMETRY_BATTERY_PERIOD_MS;
        if (with_battery) {
            since_battery_ms = 0;
        }
        send_frame(mode, with_battery);
    }
}

void telemetry_init(void)
{
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 4, NULL, 0);
}
