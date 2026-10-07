// =============================================================================
//  main.c - GRUZIK5.0 line follower firmware (ESP32-S3, ESP-IDF)
// =============================================================================
//
//  Initialises the hardware drivers, then starts the tasks:
//
//    core 1  control      1 kHz control loop (highest application priority)
//    core 0  link_rx/tx   Wi-Fi TCP command link
//            telemetry    10 Hz data stream to the app
//            map_writer   route recording to flash
//            console      USB commands and status report
// =============================================================================

#include "analog.h"
#include "commands.h"
#include "console.h"
#include "encoders.h"
#include "esp_log.h"
#include "imu.h"
#include "line_sensors.h"
#include "link.h"
#include "motors.h"
#include "robot.h"
#include "route_map.h"
#include "storage.h"
#include "telemetry.h"

static const char *TAG = "gruzik5";

static void on_link_connect(void)
{
    telemetry_send_hello();
}

static void on_link_disconnect(void)
{
    robot_lock();
    robot_set_telemetry(TELEMETRY_OFF);
    robot_unlock();
}

void app_main(void)
{
    // Motor drivers first: their wake-up handshake needs the battery (VM)
    // present, and the bridges must be defined before anything else runs.
    ESP_ERROR_CHECK(motors_init());
    ESP_ERROR_CHECK(analog_init());
    line_sensors_init();
    ESP_ERROR_CHECK(encoders_init());
    ESP_ERROR_CHECK(imu_init());
    if (storage_init() != ESP_OK) {
        ESP_LOGE(TAG, "flash filesystem unavailable, maps disabled");
    }
    route_map_init();

    robot_init();
    console_init();
    telemetry_init();
    ESP_ERROR_CHECK(link_init(commands_execute, on_link_connect, on_link_disconnect));
    ESP_LOGI(TAG, "ready");
}
