// =============================================================================
//  robot.c - robot state and the 1 kHz control loop
// =============================================================================
//
//  This module is the core of the firmware. It
//    * keeps track of what the robot is doing (stopped, line following,
//      mapping, map playback, joystick, tyre cleaning),
//    * starts and stops runs in response to commands from the app,
//    * runs one control tick every millisecond:
//      sensors -> encoders -> odometry -> controller -> motors -> LED.
//
//  Concurrency
//  -----------
//  The control tick runs in its own FreeRTOS task on core 1, while commands
//  arrive from the Wi-Fi and USB tasks on core 0. A single mutex
//  (robot_lock / robot_unlock) protects all robot state, so a command can
//  never change e.g. the PID gains halfway through a tick. The tick holds
//  the lock for its whole duration (~0.5 ms); commands hold it only while
//  they change state.
// =============================================================================

#include "robot.h"

#include <stdio.h>
#include "battery.h"
#include "board.h"
#include "driver/gpio.h"
#include "encoders.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu.h"
#include "line_follower.h"
#include "line_sensors.h"
#include "link.h"
#include "motors.h"
#include "odometry.h"
#include "route_map.h"

// --- Robot state --------------------------------------------------------------

static SemaphoreHandle_t s_lock;                             // guards everything below
static robot_mode_t s_mode = ROBOT_STOPPED;                  // what the robot does now
static robot_selection_t s_selection = ROBOT_SELECT_NORMAL;  // what "Start" will run
static telemetry_mode_t s_telemetry = TELEMETRY_OFF;
static uint16_t s_line_raw[LINE_SENSOR_COUNT];               // latest line sensor readings
static float s_clean_speed = 170.0f;                         // tyre cleaning PWM
static int64_t s_finish_at_us;   // end of the PID run-out after a playback (0 = none)
static robot_timing_t s_timing;  // tick timing statistics for the USB log

// --- State access -------------------------------------------------------------

void robot_lock(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
}

void robot_unlock(void)
{
    xSemaphoreGive(s_lock);
}

robot_mode_t robot_mode(void)
{
    return s_mode;
}

const uint16_t *robot_line_raw(void)
{
    return s_line_raw;
}

void robot_set_telemetry(telemetry_mode_t mode)
{
    s_telemetry = mode;
}

telemetry_mode_t robot_telemetry(void)
{
    return s_telemetry;
}

robot_timing_t robot_take_timing(void)
{
    const robot_timing_t timing = s_timing;
    s_timing.tick_max_us = 0;  // the next read reports the maximum from now on
    return timing;
}

// --- Starting and stopping runs -----------------------------------------------

// Every run starts from scratch: encoder counts cleared, pose at (0, 0, 0 deg).
static void reset_run(void)
{
    encoders_reset();
    odometry_reset();
    s_finish_at_us = 0;
}

// Stops everything immediately, without reporting to the app.
static void halt(void)
{
    route_map_end_recording();
    line_follower_stop();
    motors_stop();
    s_mode = ROBOT_STOPPED;
    s_finish_at_us = 0;
}

static void report_stop(void)
{
    char msg[48];
    snprintf(msg, sizeof(msg), "Stop\r\nBattery = %.2f V\r\n", battery_voltage());
    link_send(msg);
}

// Start report for the app. This is also where the speed level is latched:
// a PWM multiplier derived from the battery voltage at the moment of the
// start, so a weaker pack still gives the same driving speed.
static void report_start(const char *kind)
{
    motors_set_speed_level(battery_speed_level());
    char imu[96];
    imu_format_status(imu, sizeof(imu));
    char msg[256];
    snprintf(msg, sizeof(msg), "STARTING,%s\r\n%sStart\r\nBattery = %.2f V\r\nSpeed_level = %.2f\r\n%s",
             kind, battery_voltage() < BATTERY_LOW_WARNING_V ? "! Low Battery !\r\n" : "",
             battery_voltage(), motors_speed_level(), imu);
    link_send(msg);
}

// Plain line following on the PID controller.
void robot_start_normal(void)
{
    route_map_end_recording();
    reset_run();
    line_follower_start();
    s_mode = ROBOT_NORMAL;
    report_start("normal");
}

// Line following on PID while the route is recorded to GRUZIK.txt.
void robot_start_mapping(void)
{
    reset_run();
    if (!route_map_begin_recording(odometry_get())) {
        link_send("MAP_ERROR,open_write,GRUZIK.txt,1\r\n");
        return;
    }
    line_follower_start();
    s_mode = ROBOT_MAPPING;
    report_start("mapping");
}

// Driving the stored route from map.txt (odometry plus a line correction).
void robot_start_playback(void)
{
    route_map_end_recording();
    if (!route_map_begin_playback()) {
        link_send("MAP_ERROR,open_playback,map.txt,4\r\n");
        return;
    }
    reset_run();
    line_follower_start();
    s_mode = ROBOT_PLAYBACK;
    report_start("playback");
}

// "Start" in the app runs the mode chosen earlier with Mode=P/M/U.
void robot_start_selected(void)
{
    switch (s_selection) {
    case ROBOT_SELECT_NORMAL: robot_start_normal(); break;
    case ROBOT_SELECT_MAPPING: robot_start_mapping(); break;
    case ROBOT_SELECT_PLAYBACK: robot_start_playback(); break;
    }
}

void robot_select(robot_selection_t selection)
{
    if (s_mode != ROBOT_STOPPED) {
        link_send("Stop robot before changing state\r\n");
        return;
    }
    s_selection = selection;
    static const char *const names[] = {"State: PID\r\n", "State: Mapping\r\n",
                                        "State: UnMapping\r\n"};
    link_send(names[selection]);
}

void robot_stop(void)
{
    halt();
    report_stop();
}

// --- Manual control -------------------------------------------------------------

// Joystick input in PWM units (+-MOTOR_PWM_MAX). Zero on both wheels stops.
void robot_manual(float left_pwm, float right_pwm)
{
    line_follower_stop();
    if (left_pwm > -0.5f && left_pwm < 0.5f && right_pwm > -0.5f && right_pwm < 0.5f) {
        motors_stop();
        s_mode = ROBOT_STOPPED;
        return;
    }
    s_mode = ROBOT_MANUAL;
    motors_drive(left_pwm, right_pwm);
}

void robot_set_clean_speed(float pwm)
{
    s_clean_speed = pwm;
}

// Tyre cleaning: both wheels spin while the button in the app is held.
void robot_clean(bool on)
{
    line_follower_stop();
    if (on) {
        s_mode = ROBOT_CLEANING;
        motors_drive(s_clean_speed, s_clean_speed);
        link_send("CLEAN,start\r\n");
    } else {
        motors_stop();
        s_mode = ROBOT_STOPPED;
        link_send("CLEAN,stop\r\n");
    }
}

// --- Control tick ---------------------------------------------------------------

// After the last map point the robot keeps following the line on PID for a
// short while to cross the finish cleanly, then stops.
static void finish_playback(int64_t now_us)
{
    s_mode = ROBOT_NORMAL;
    line_follower_start();
    s_finish_at_us = now_us + MAP_FINISH_PID_MS * 1000LL;
    link_send("MAP_PLAYBACK_DONE\r\n");
}

// Status LED:
//   driving                          -> blinks at 1 Hz
//   no phone connected               -> blinks at 3 Hz
//   phone connected, robot stopped   -> solid
static void update_status_led(uint32_t now_ms)
{
    bool on;
    if (s_mode != ROBOT_STOPPED) {
        on = (now_ms / LED_DRIVING_TOGGLE_MS) % 2 == 0;
    } else if (!link_connected()) {
        on = (now_ms / LED_SEARCH_TOGGLE_MS) % 2 == 0;
    } else {
        on = true;
    }
    gpio_set_level(PIN_STATUS_LED, on);
}

// Driving in the line modes (PID, mapping, playback).
// line_fresh: a complete sweep of all 16 sensors finished in this tick.
static void drive_line(int64_t now_us, uint32_t now_ms, bool line_fresh)
{
    // The line PID only runs on a fresh, complete sensor sweep (every 2 ms).
    // Its result is kept because playback steers every 1 ms and uses the
    // latest line correction.
    static line_output_t out;
    if (line_fresh) {
        out = line_follower_update(s_line_raw, now_ms);
    }

    if (s_mode == ROBOT_PLAYBACK) {
        float left, right;
        if (route_map_playback_step(odometry_get(), &out, line_follower_status(), &left, &right)) {
            motors_drive(left, right);
        } else {
            finish_playback(now_us);
        }
        return;
    }

    if (line_fresh && out.valid) {
        motors_drive(out.left, out.right);
    }

    // Mapping: a point is recorded every 40 ms; once the route closes into a
    // loop near the start, the robot stops.
    if (s_mode == ROBOT_MAPPING) {
        const float speed = 0.5f * (encoders_left()->mps + encoders_right()->mps);
        if (route_map_record_tick(odometry_get(), speed, line_follower_status()->error, now_ms)) {
            line_follower_stop();
            motors_stop();
            s_mode = ROBOT_STOPPED;
            char msg[40];
            snprintf(msg, sizeof(msg), "MAP_AUTO_CLOSED,%u\r\n", route_map_recorded_points());
            link_send(msg);
        }
    }

    // End of the PID run-out after a playback.
    if (s_finish_at_us != 0 && now_us >= s_finish_at_us) {
        robot_stop();
    }
}

// One control tick. The order matters: measurements first, then the
// computations, outputs last.
static void control_tick(int64_t now_us, float dt_s)
{
    const uint32_t now_ms = (uint32_t)(now_us / 1000);

    // 1. Measurements.
    const bool stationary = s_mode == ROBOT_STOPPED && encoders_left()->delta == 0 &&
                            encoders_right()->delta == 0;
    imu_update(dt_s, stationary);  // the gyro re-learns its drift while standing still
    const bool line_fresh = line_sensors_scan_step(s_line_raw);
    encoders_update(dt_s);

    // 2. Robot pose from the encoders (fused with the gyro when present).
    odometry_update(encoders_left()->delta_m, encoders_right()->delta_m,
                    imu_take_yaw_delta_rad(), imu_state()->mode != IMU_MODE_NONE);

    // 3. Battery every 250 ms (a single conversion, to keep the tick short).
    static uint32_t battery_divider;
    if (++battery_divider >= BATTERY_PERIOD_MS / CONTROL_PERIOD_MS) {
        battery_divider = 0;
        battery_update();
    }

    // 4. Controller and motors. In manual and cleaning mode the command sets
    //    the motors directly, so the tick has nothing to do here.
    if (s_mode == ROBOT_NORMAL || s_mode == ROBOT_MAPPING || s_mode == ROBOT_PLAYBACK) {
        drive_line(now_us, now_ms, line_fresh);
    }

    // 5. Status LED.
    update_status_led(now_ms);
}

// FreeRTOS task running the control tick every millisecond.
static void control_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    int64_t previous = esp_timer_get_time();
    for (;;) {
        // xTaskDelayUntil wakes the task on a fixed grid (not "1 ms after the
        // previous tick ended"), so delays never accumulate.
        if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS)) == pdFALSE) {
            ++s_timing.overruns;  // the previous tick ran late
        }
        const int64_t start = esp_timer_get_time();

        // Speeds, odometry and gyro integration use the measured elapsed time,
        // so a single late tick does not distort them.
        float dt_s = (start - previous) * 1e-6f;
        previous = start;
        if (dt_s <= 0.0f || dt_s > 0.02f) {
            dt_s = CONTROL_DT_S;
        }

        robot_lock();
        control_tick(start, dt_s);
        robot_unlock();

        const uint32_t took = (uint32_t)(esp_timer_get_time() - start);
        if (took > s_timing.tick_max_us) {
            s_timing.tick_max_us = took;
        }
    }
}

void robot_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    const gpio_config_t led = {
        .pin_bit_mask = 1ULL << PIN_STATUS_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led);
    battery_update();  // so the first report already has a voltage

    // Core 1 belongs to the control loop alone; Wi-Fi, files and USB run on
    // core 0. High priority: the tick preempts everything but the system.
    xTaskCreatePinnedToCore(control_task, "control", 6144, NULL,
                            configMAX_PRIORITIES - 2, NULL, 1);
}
