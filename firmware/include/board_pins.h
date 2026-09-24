#pragma once

#include <Arduino.h>

namespace BoardPins {
// Shared SPI bus. U7 (TOP) is populated; U10 (BOTTOM) is optional.
constexpr uint8_t imuMosi = 11;
constexpr uint8_t imuSclk = 12;
constexpr uint8_t imuMiso = 13;
constexpr uint8_t imuU7Cs = 7;
constexpr uint8_t imuU7Int = 6;
constexpr uint8_t imuU10Cs = 9;
constexpr uint8_t imuU10Int = 8;

// Sensor daughterboard: 74HC4067 Z/S0/S1/S2/S3/E.
constexpr uint8_t lineMuxAdc = 10;
constexpr uint8_t lineMuxS0 = 14;
constexpr uint8_t lineMuxS1 = 15;
constexpr uint8_t lineMuxS2 = 16;
constexpr uint8_t lineMuxS3 = 17;
constexpr uint8_t lineMuxEnable = 18;

constexpr uint8_t batteryAdc = 1;
constexpr uint8_t statusLed = 47;

constexpr uint8_t motorLeftPwm = 41;
constexpr uint8_t motorLeftDir = 42;
constexpr uint8_t motorRightPwm = 21;
constexpr uint8_t motorRightDir = 48;
constexpr uint8_t motorSleepN = 39;
constexpr uint8_t motorDriveOff = 40;

constexpr uint8_t encoderLeftA = 37;
constexpr uint8_t encoderLeftB = 36;
constexpr uint8_t encoderLeftIndex = 38;
constexpr uint8_t encoderRightA = 33;
constexpr uint8_t encoderRightB = 34;
constexpr uint8_t encoderRightIndex = 35;
}  // namespace BoardPins

