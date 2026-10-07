// =============================================================================
//  imu.c - ICM-42688-P gyro driver over SPI
// =============================================================================
//
//  Only the Z gyro (yaw rate) and the temperature are used. Both sensors share
//  SPI2; the driver toggles each chip select. Transfers are polled because
//  they are tiny (3 bytes) and run inside the 1 ms control tick.
// =============================================================================

#include "imu.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "config.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "imu";

// Registers (bank 0 unless noted).
#define REG_BANK_SELECT     0x76
#define REG_WHO_AM_I        0x75
#define REG_PWR_MGMT0       0x4E
#define REG_GYRO_CONFIG0    0x4F
#define REG_GYRO_Z1         0x29    // GYRO_DATA_Z1 (high byte), Z0 follows
#define REG_TEMP1           0x1D    // TEMP_DATA1 (high byte), TEMP_DATA0 follows
#define WHO_AM_I_VALUE      0x47

#define GYRO_2000DPS_LSB    16.4f   // LSB per deg/s at +-2000 dps
#define SPI_CLOCK_HZ        1000000
#define CALIBRATION_SAMPLES 300
#define TEMP_PERIOD_TICKS   1000    // temperature once per second

#ifdef CONFIG_GRUZIK_IMU_ENABLED
static const bool IMU_ENABLED = true;
#else
static const bool IMU_ENABLED = false;
#endif

typedef struct {
    gpio_num_t cs;
    float yaw_sign;         // -1 for the upside-down sensor
    spi_device_handle_t dev;
    bool ok;
    uint8_t who_am_i;
    float raw_dps;
    float bias_dps;
} icm_t;

static icm_t s_sensors[2] = {
    {.cs = PIN_IMU_U7_CS, .yaw_sign = +1.0f},
    {.cs = PIN_IMU_U10_CS, .yaw_sign = -1.0f},
};
static imu_state_t s_state;
static float s_pending_yaw_rad;

// --- Register access --------------------------------------------------------------

static esp_err_t icm_write(icm_t *icm, uint8_t reg, uint8_t value)
{
    spi_transaction_t t = {
        .length = 16,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {reg & 0x7F, value},     // bit 7 clear = write
    };
    return spi_device_polling_transmit(icm->dev, &t);
}

static esp_err_t icm_read(icm_t *icm, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx[8] = {reg | 0x80};           // bit 7 set = read
    uint8_t rx[8] = {0};
    if (len + 1 > sizeof(tx)) {
        return ESP_ERR_INVALID_SIZE;
    }
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(icm->dev, &t), TAG, "read");
    memcpy(data, rx + 1, len);              // rx[0] is clocked in during the address
    return ESP_OK;
}

// Reads a big-endian 16-bit register pair.
static bool icm_read_i16(icm_t *icm, uint8_t reg, int16_t *value)
{
    uint8_t bytes[2];
    if (icm_read(icm, reg, bytes, 2) != ESP_OK) {
        return false;
    }
    *value = (int16_t)((bytes[0] << 8) | bytes[1]);
    return true;
}

// --- Setup ----------------------------------------------------------------------------

// Checks WHO_AM_I and configures the gyro. Returns false if no sensor answers.
static bool icm_begin(icm_t *icm)
{
    const spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = icm->cs,
        .queue_size = 1,
    };
    if (spi_bus_add_device(SPI2_HOST, &dev_cfg, &icm->dev) != ESP_OK) {
        return false;
    }
    if (icm_write(icm, REG_BANK_SELECT, 0x00) != ESP_OK ||
        icm_read(icm, REG_WHO_AM_I, &icm->who_am_i, 1) != ESP_OK ||
        icm->who_am_i != WHO_AM_I_VALUE) {
        return false;
    }
    icm_write(icm, REG_PWR_MGMT0, 0x1C);       // gyro + accel in low-noise mode
    vTaskDelay(pdMS_TO_TICKS(100));            // gyro start-up time
    icm_write(icm, REG_BANK_SELECT, 0x01);     // bank 1: anti-alias / notch setup
    icm_write(icm, 0x03, 0x00);
    icm_write(icm, 0x0B, 0xA0);
    icm_write(icm, 0x0C, 0x0D);
    icm_write(icm, 0x13, 0x04);
    icm_write(icm, REG_BANK_SELECT, 0x00);
    icm_write(icm, REG_GYRO_CONFIG0, 0x06);    // +-2000 dps, 1 kHz ODR
    return true;
}

// Averages the zero-rate output while the robot stands still.
static void icm_calibrate(icm_t *icm)
{
    float sum = 0.0f;
    int valid = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        int16_t raw;
        if (icm_read_i16(icm, REG_GYRO_Z1, &raw)) {
            sum += raw / GYRO_2000DPS_LSB;
            ++valid;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (valid > 0) {
        icm->bias_dps = sum / valid;
    }
}

esp_err_t imu_init(void)
{
    if (!IMU_ENABLED) {
        s_state.mode = IMU_MODE_NONE;
        return ESP_OK;
    }
    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_IMU_MOSI,
        .miso_io_num = PIN_IMU_MISO,
        .sclk_io_num = PIN_IMU_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED), TAG, "bus");
    int count = 0;
    for (int i = 0; i < 2; ++i) {
        icm_t *icm = &s_sensors[i];
        icm->ok = icm_begin(icm);
        if (icm->ok) {
            icm_calibrate(icm);
            ++count;
        }
    }
    s_state.mode = (imu_mode_t)count;
    ESP_LOGI(TAG, "mode=%d U7(who=0x%02X) U10(who=0x%02X)", count,
             s_sensors[0].who_am_i, s_sensors[1].who_am_i);
    return ESP_OK;
}

// --- Runtime ------------------------------------------------------------------------

void imu_update(float dt_s, bool robot_stationary)
{
    if (s_state.mode == IMU_MODE_NONE) {
        return;
    }
    float raw = 0.0f, bias = 0.0f, rate = 0.0f;
    int valid = 0;
    for (int i = 0; i < 2; ++i) {
        icm_t *icm = &s_sensors[i];
        int16_t value;
        if (!icm->ok || !icm_read_i16(icm, REG_GYRO_Z1, &value)) {
            continue;
        }
        icm->raw_dps = value / GYRO_2000DPS_LSB;
        float corrected = (icm->raw_dps - icm->bias_dps) * icm->yaw_sign;
        // Standing still and reading almost zero: nudge the bias towards the
        // reading so slow thermal drift does not accumulate as yaw.
        if (robot_stationary && fabsf(corrected) < GYRO_STATIONARY_DPS) {
            icm->bias_dps += (icm->raw_dps - icm->bias_dps) * GYRO_BIAS_LEARN_RATE;
            corrected = (icm->raw_dps - icm->bias_dps) * icm->yaw_sign;
        }
        raw += icm->raw_dps * icm->yaw_sign;
        bias += icm->bias_dps * icm->yaw_sign;
        rate += corrected;
        ++valid;
    }
    if (valid == 0) {
        s_state.gyro_z_raw_dps = s_state.gyro_z_bias_dps = s_state.gyro_z_dps = 0.0f;
        return;
    }
    s_state.gyro_z_raw_dps = raw / valid;
    s_state.gyro_z_bias_dps = bias / valid;
    s_state.gyro_z_dps = rate / valid;
    s_pending_yaw_rad += s_state.gyro_z_dps * (float)M_PI / 180.0f * dt_s;

    static int temp_divider;
    if (++temp_divider >= TEMP_PERIOD_TICKS) {
        temp_divider = 0;
        int16_t t;
        if (icm_read_i16(&s_sensors[s_sensors[0].ok ? 0 : 1], REG_TEMP1, &t)) {
            s_state.temperature_c = t / 132.48f + 25.0f;   // datasheet formula
        }
    }
}

float imu_take_yaw_delta_rad(void)
{
    const float delta = s_pending_yaw_rad;
    s_pending_yaw_rad = 0.0f;
    return delta;
}

const imu_state_t *imu_state(void)
{
    return &s_state;
}

void imu_format_status(char *buf, size_t len)
{
    if (!IMU_ENABLED) {
        snprintf(buf, len, "IMU_MODE,0,U7=0,U10=0,DISABLED=1\r\n");
        return;
    }
    snprintf(buf, len, "IMU_MODE,%d,U7=%d,U10=%d,WHO7=0x%x,WHO10=0x%x\r\n",
             s_state.mode, s_sensors[0].ok, s_sensors[1].ok,
             s_sensors[0].who_am_i, s_sensors[1].who_am_i);
}
