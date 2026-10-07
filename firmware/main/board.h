#pragma once

// =============================================================================
//  board.h - GRUZIK5.0 pin map
// =============================================================================
//
//  Source: schematic "GRUZIK5.0" V1.0, ESP32-S3-MINI-1-N8 module.
//  A board revision only needs changes in this file.
// =============================================================================

#include "driver/gpio.h"

// --- IMU: ICM-42688-P on a shared SPI bus --------------------------------------
// U7 sits on the top side (always fitted), U10 on the bottom side (optional).
#define PIN_IMU_MOSI        GPIO_NUM_11
#define PIN_IMU_SCLK        GPIO_NUM_12
#define PIN_IMU_MISO        GPIO_NUM_13
#define PIN_IMU_U7_CS       GPIO_NUM_7
#define PIN_IMU_U7_INT      GPIO_NUM_6
#define PIN_IMU_U10_CS      GPIO_NUM_9
#define PIN_IMU_U10_INT     GPIO_NUM_8

// --- Line sensors: Sensorx16 board with a 74HC4067 multiplexer -----------------
// Z = shared analog output, S0..S3 = channel select, E = enable (active low).
#define PIN_LINE_MUX_Z      GPIO_NUM_10
#define PIN_LINE_MUX_S0     GPIO_NUM_14
#define PIN_LINE_MUX_S1     GPIO_NUM_15
#define PIN_LINE_MUX_S2     GPIO_NUM_16
#define PIN_LINE_MUX_S3     GPIO_NUM_17
#define PIN_LINE_MUX_E      GPIO_NUM_18

// --- Battery sense: R28 100k / R29 22k divider from VIN ------------------------
#define PIN_BATTERY_ADC     GPIO_NUM_1

// --- Status LED (LED3) -----------------------------------------------------------
#define PIN_STATUS_LED      GPIO_NUM_47

// --- Motors: two DRV8245H-Q1 in PH/EN mode (MODE pin tied to GND) --------------
// EN/IN1 = PWM (speed), PH/IN2 = direction.
#define PIN_MOTOR_L_PWM     GPIO_NUM_41
#define PIN_MOTOR_L_DIR     GPIO_NUM_42
#define PIN_MOTOR_R_PWM     GPIO_NUM_21
#define PIN_MOTOR_R_DIR     GPIO_NUM_48
// Shared by both drivers. nFAULT is not routed to the ESP32.
#define PIN_MOTOR_NSLEEP    GPIO_NUM_39
#define PIN_MOTOR_DRVOFF    GPIO_NUM_40

// --- Encoders IEP3-128: quadrature A/B plus index pulse I ----------------------
#define PIN_ENC_L_A         GPIO_NUM_37
#define PIN_ENC_L_B         GPIO_NUM_36
#define PIN_ENC_L_I         GPIO_NUM_38
#define PIN_ENC_R_A         GPIO_NUM_33
#define PIN_ENC_R_B         GPIO_NUM_34
#define PIN_ENC_R_I         GPIO_NUM_35
