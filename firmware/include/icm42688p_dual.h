#pragma once

#include <Arduino.h>
#include <SPI.h>

// ESP32-S3 port of the minimal, proven ICM-42688-P driver from GRUZIK4.0.
// The register sequence and scale are intentionally kept identical to the
// STM32 implementation. The only extension is a per-device chip-select and
// an orientation sign so two sensors can share one SPI bus.
class Icm42688p {
 public:
  Icm42688p(SPIClass &spi, uint8_t chipSelect, float yawOrientationSign)
      : spi_(spi), chipSelect_(chipSelect),
        yawOrientationSign_(yawOrientationSign) {}

  // Return values match the GRUZIK4.0 driver:
  // 1 = initialized, 0 = wrong/no WHO_AM_I, 2 = bus/configuration failure.
  uint8_t begin() {
    initialized_ = false;
    whoAmI_ = 0;
    gyroZRawDps_ = 0.0f;
    gyroZBiasDps_ = 0.0f;
    gyroZDps_ = 0.0f;
    consecutiveErrors_ = 0;

    pinMode(chipSelect_, OUTPUT);
    deselect();
    delay(10);

    if (!selectBank(0x00)) return 2;
    if (!readRegisters(kWhoAmIRegister, &whoAmI_, 1)) return 2;
    if (whoAmI_ != kWhoAmIValue) return 0;

    if (!writeRegister(kPowerManagement0Register, 0x1C)) return 2;
    delay(100);

    if (!selectBank(0x01)) return 2;
    if (!writeRegister(0x03, 0x00)) return 2;
    if (!writeRegister(0x0B, 0xA0)) return 2;
    if (!writeRegister(0x0C, 0x0D)) return 2;
    if (!writeRegister(0x13, 0x04)) return 2;

    if (!selectBank(0x00)) return 2;
    if (!writeRegister(kGyroConfig0Register, 0x06)) return 2;

    initialized_ = true;
    return 1;
  }

  bool readGyroZ() {
    if (!initialized_) return false;
    uint8_t bytes[2] = {0, 0};
    if (!readRegisters(kGyroZ1Register, bytes, sizeof(bytes))) {
      ++consecutiveErrors_;
      return false;
    }

    const int16_t raw = static_cast<int16_t>(
        (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
    gyroZRawDps_ = static_cast<float>(raw) / kGyro2000DpsLsb;
    gyroZDps_ = (gyroZRawDps_ - gyroZBiasDps_) * yawOrientationSign_;
    consecutiveErrors_ = 0;
    return true;
  }

  bool readTemperature() {
    if (!initialized_) return false;
    uint8_t bytes[2] = {0, 0};
    if (!readRegisters(kTemperature1Register, bytes, sizeof(bytes))) return false;
    const int16_t raw = static_cast<int16_t>(
        (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
    temperatureC_ = static_cast<float>(raw) / 132.48f + 25.0f;
    return true;
  }

  bool calibrateGyroZ(uint16_t samples, uint32_t sampleDelayMs) {
    if (!initialized_ || samples == 0) return false;

    const float previousBias = gyroZBiasDps_;
    gyroZBiasDps_ = 0.0f;
    float sum = 0.0f;
    uint16_t validSamples = 0;
    for (uint16_t sample = 0; sample < samples; ++sample) {
      if (readGyroZ()) {
        sum += gyroZRawDps_;
        ++validSamples;
      }
      delay(sampleDelayMs);
    }

    if (validSamples == 0) {
      gyroZBiasDps_ = previousBias;
      return false;
    }
    gyroZBiasDps_ = sum / static_cast<float>(validSamples);
    gyroZDps_ = 0.0f;
    return true;
  }

  void updateGyroZBias(float learnRate) {
    gyroZBiasDps_ += (gyroZRawDps_ - gyroZBiasDps_) * learnRate;
    gyroZDps_ = (gyroZRawDps_ - gyroZBiasDps_) * yawOrientationSign_;
  }

  bool initialized() const { return initialized_; }
  uint8_t whoAmI() const { return whoAmI_; }
  uint16_t consecutiveErrors() const { return consecutiveErrors_; }
  float gyroZRawDps() const { return gyroZRawDps_; }
  float gyroZBiasDps() const { return gyroZBiasDps_; }
  float gyroZDps() const { return gyroZDps_; }
  float temperatureC() const { return temperatureC_; }
  float orientationSign() const { return yawOrientationSign_; }

 private:
  static constexpr uint8_t kBankSelectRegister = 0x76;
  static constexpr uint8_t kWhoAmIRegister = 0x75;
  static constexpr uint8_t kPowerManagement0Register = 0x4E;
  static constexpr uint8_t kGyroConfig0Register = 0x4F;
  static constexpr uint8_t kGyroZ1Register = 0x29;
  static constexpr uint8_t kTemperature1Register = 0x1D;
  static constexpr uint8_t kWhoAmIValue = 0x47;
  static constexpr float kGyro2000DpsLsb = 16.4f;
  static constexpr uint32_t kSpiClockHz = 1000000;

  void select() { digitalWrite(chipSelect_, LOW); }
  void deselect() { digitalWrite(chipSelect_, HIGH); }

  bool writeRegister(uint8_t reg, uint8_t value) {
    spi_.beginTransaction(SPISettings(kSpiClockHz, MSBFIRST, SPI_MODE0));
    select();
    spi_.transfer(reg & 0x7F);
    spi_.transfer(value);
    deselect();
    spi_.endTransaction();
    return true;
  }

  bool readRegisters(uint8_t reg, uint8_t *data, size_t length) {
    spi_.beginTransaction(SPISettings(kSpiClockHz, MSBFIRST, SPI_MODE0));
    select();
    spi_.transfer(reg | 0x80);
    for (size_t index = 0; index < length; ++index) {
      data[index] = spi_.transfer(0x00);
    }
    deselect();
    spi_.endTransaction();
    return true;
  }

  bool selectBank(uint8_t bank) {
    return writeRegister(kBankSelectRegister, bank);
  }

  SPIClass &spi_;
  uint8_t chipSelect_;
  float yawOrientationSign_;
  bool initialized_ = false;
  uint8_t whoAmI_ = 0;
  uint16_t consecutiveErrors_ = 0;
  float gyroZRawDps_ = 0.0f;
  float gyroZBiasDps_ = 0.0f;
  float gyroZDps_ = 0.0f;
  float temperatureC_ = 0.0f;
};
