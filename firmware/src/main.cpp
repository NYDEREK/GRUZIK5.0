#include <Arduino.h>
#include <LittleFS.h>
#include <NimBLEDevice.h>
#include <SPI.h>
#include <driver/adc.h>
#include <math.h>
#include <vector>

#include "board_pins.h"
#include "icm42688p_dual.h"
#include "line_follower.h"

namespace Config {
constexpr char bleName[] = "gruzik 5.0";
constexpr char bleServiceUuid[] = "8f1d0001-8a6b-4dd2-9b52-8ea36a45f001";
constexpr char bleCommandUuid[] = "8f1d0003-8a6b-4dd2-9b52-8ea36a45f001";
constexpr char bleTextUuid[] = "8f1d0004-8a6b-4dd2-9b52-8ea36a45f001";
// IMU is intentionally out of scope for the current bring-up. Odometry falls
// back to encoder yaw until the SPI hardware is verified separately.
constexpr bool imuEnabled = false;

constexpr uint32_t imuPeriodUs = 1000;
constexpr uint32_t lineScanPeriodMs = 1;
constexpr uint32_t encoderPeriodMs = 20;
constexpr uint32_t batteryPeriodMs = 250;
constexpr uint32_t telemetryPeriodMs = 100;
constexpr uint32_t batteryTelemetryPeriodMs = 1000;
constexpr uint32_t mapRecordPeriodMs = 40;
constexpr uint32_t mapFinishPidMs = 1750;
constexpr uint32_t statusLedStoppedToggleMs = 500;

// The 74HC4067 channels are wired in two banks on the Sensorx16 board:
// Y0..Y7 = SENSOR8..SENSOR15 and Y8..Y15 = SENSOR0..SENSOR7.
// Keep lineRaw[] in logical SENSOR0..SENSOR15 order, so the controller and
// BLE diagnostics see the physical numbering printed on the daughterboard.
constexpr uint8_t lineSensorByMuxChannel[16] = {
    8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7};

constexpr uint8_t motorPwmLimitPercent = 70;
constexpr uint32_t motorPwmFrequency = 20000;
constexpr uint8_t motorPwmResolution = 10;
constexpr uint8_t motorLeftPwmChannel = 0;
constexpr uint8_t motorRightPwmChannel = 1;
constexpr float gruzik4PwmMaximum = 286.0f;

constexpr float batteryDivider = 122.0f / 22.0f;
constexpr float batteryEmptyVoltage = 9.9f;
constexpr float batteryFullVoltage = 12.6f;

constexpr uint16_t encoderCountsPerWheelRevolution = 2560;
constexpr int8_t leftEncoderDirection = -1;
constexpr int8_t rightEncoderDirection = 1;
constexpr float wheelCircumferenceM = 0.07225663f;
constexpr float wheelBaseM = 0.195f;
constexpr float sensorOffsetM = 0.220f;
constexpr float odometryGyroWeight = 0.96f;
constexpr float gyroStationaryDps = 2.0f;
constexpr float gyroBiasLearnRate = 0.0005f;

constexpr float mapTargetRadiusM = 0.075f;
constexpr float mapAdvanceLateralM = 0.120f;
constexpr float mapCloseRadiusM = 0.030f;
constexpr float mapCloseMinimumDistanceM = 0.450f;
constexpr uint16_t mapCloseMinimumPoints = 8;
constexpr float playbackLineWeight = 0.18f;
constexpr float playbackLineCorrectionLimit = 35.0f;
constexpr float playbackOdomCorrectionLimit = 115.0f;
constexpr float playbackCorrectionLimit = 140.0f;
constexpr float playbackLookaheadBaseM = 0.120f;
constexpr float playbackLookaheadSpeedM = 0.140f;
constexpr float playbackCrosstrackLimitM = 0.180f;
constexpr float playbackDerivativeAlpha = 0.28f;
constexpr float playbackTurnSpeedReduction = 0.35f;
constexpr float playbackLostLineSpeedMultiplier = 0.85f;

constexpr char recordedMapPath[] = "/GRUZIK.txt";
constexpr char optimizedMapPath[] = "/map.txt";
}  // namespace Config

enum class ImuMode : uint8_t { None = 0, Single = 1, Dual = 2 };
enum class RobotMode : uint8_t {
  Stopped,
  Normal,
  Mapping,
  Playback,
  Manual,
  Cleaning,
};
enum class SelectedMode : uint8_t { Normal, Mapping, Playback };
enum class TelemetryMode : uint8_t { Off, Odom, Debug };

struct MotorState {
  int8_t direction = 0;
  uint8_t speedPercent = 0;
};

struct EncoderState {
  volatile int32_t count = 0;
  volatile uint32_t indexPulses = 0;
  volatile uint8_t previousState = 0;
  int32_t lastCount = 0;
  int32_t lastDelta = 0;
  float rpm = 0.0f;
  float metersPerSecond = 0.0f;
};

struct OdometryState {
  float xM = 0.0f;
  float yM = 0.0f;
  float yawRad = 0.0f;
  float gyroYawRad = 0.0f;
  float encoderYawRad = 0.0f;
  float totalDistanceM = 0.0f;
};

struct MapPoint {
  float x = 0.0f;
  float y = 0.0f;
  float speed = 80.0f;
};

LineFollowerController lineFollower;
MotorState leftMotor;
MotorState rightMotor;
EncoderState leftEncoder;
EncoderState rightEncoder;
OdometryState odometry;

// U7 is on the top side. U10 is the optional, oppositely mounted sensor on
// the bottom side, therefore its physical yaw axis is reversed.
Icm42688p imuU7(SPI, BoardPins::imuU7Cs, +1.0f);
Icm42688p imuU10(SPI, BoardPins::imuU10Cs, -1.0f);
ImuMode imuMode = ImuMode::None;
float gyroZRawDps = 0.0f;
float gyroZBiasDps = 0.0f;
float gyroZDps = 0.0f;
float gyroTemperatureC = 0.0f;
float pendingGyroDeltaRad = 0.0f;
uint32_t lastImuUs = 0;

uint16_t lineRaw[16] = {};
uint32_t lastLineScanMs = 0;
uint8_t nextLineMuxChannel = 0;
float batteryVoltage = 0.0f;
float batteryPercent = 0.0f;
uint32_t lastBatteryMs = 0;
uint32_t lastEncoderMs = 0;
bool adcReady = false;
uint16_t adcLatest[10]{};
uint16_t adcSeenMask = 0;
uint32_t adcReadOkCount = 0;
uint32_t adcReadErrorCount = 0;
uint32_t adcParsedSampleCount = 0;
uint32_t adcLastBytesRead = 0;
esp_err_t adcLastReadResult = ESP_OK;

RobotMode robotMode = RobotMode::Stopped;
SelectedMode selectedMode = SelectedMode::Normal;
TelemetryMode telemetryMode = TelemetryMode::Off;
float cleanSpeed = 170.0f;
float turbineSpeed = 0.0f;
float turbinePreparationTime = 0.0f;

float mapP = 90.0f;
float mapI = 0.0f;
float mapD = 10.0f;
float mapDefaultSpeed = 80.0f;
float mapErrorSum = 0.0f;
float mapLastError = 0.0f;
float mapFilteredDerivative = 0.0f;
std::vector<MapPoint> playbackMap;
size_t playbackTargetIndex = 0;
uint32_t playbackFinishAtMs = 0;

File recordingFile;
File uploadFile;
uint16_t uploadedPointCount = 0;
uint16_t expectedUploadPointCount = 0;
uint16_t recordedPointCount = 0;
float recordedDistanceM = 0.0f;
float mapStartX = 0.0f;
float mapStartY = 0.0f;
float mapStartSensorX = 0.0f;
float mapStartSensorY = 0.0f;
float lastMapSensorX = 0.0f;
float lastMapSensorY = 0.0f;
float previousMapLineError = 0.0f;
uint32_t lastMapRecordMs = 0;

NimBLECharacteristic *bleTextCharacteristic = nullptr;
bool bleClientConnected = false;
bool bleHelloPending = false;
uint32_t bleConnectedAtMs = 0;
String bleReceiveBuffer;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatteryTelemetryMs = 0;
uint32_t lastSerialStatusMs = 0;
bool statusLedOn = false;

float clampValue(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle <= -PI) angle += 2.0f * PI;
  return angle;
}

void writeMotorPwm(uint8_t channel, uint8_t percent) {
  const uint16_t maximumDuty = (1U << Config::motorPwmResolution) - 1U;
  ledcWrite(channel, maximumDuty * percent / 100U);
}

void setupMotors() {
  pinMode(BoardPins::motorLeftDir, OUTPUT);
  pinMode(BoardPins::motorRightDir, OUTPUT);
  pinMode(BoardPins::motorSleepN, OUTPUT);
  pinMode(BoardPins::motorDriveOff, OUTPUT);
  digitalWrite(BoardPins::motorLeftDir, LOW);
  digitalWrite(BoardPins::motorRightDir, LOW);
  digitalWrite(BoardPins::motorDriveOff, HIGH);
  digitalWrite(BoardPins::motorSleepN, LOW);
  ledcSetup(Config::motorLeftPwmChannel, Config::motorPwmFrequency,
            Config::motorPwmResolution);
  ledcSetup(Config::motorRightPwmChannel, Config::motorPwmFrequency,
            Config::motorPwmResolution);
  ledcAttachPin(BoardPins::motorLeftPwm, Config::motorLeftPwmChannel);
  ledcAttachPin(BoardPins::motorRightPwm, Config::motorRightPwmChannel);
  writeMotorPwm(Config::motorLeftPwmChannel, 0);
  writeMotorPwm(Config::motorRightPwmChannel, 0);
}

void setMotor(MotorState &state, uint8_t directionPin, uint8_t pwmChannel,
              int commandPercent) {
  commandPercent = constrain(commandPercent, -Config::motorPwmLimitPercent,
                             Config::motorPwmLimitPercent);
  const int8_t direction = commandPercent > 0 ? 1 : (commandPercent < 0 ? -1 : 0);
  const uint8_t speed = abs(commandPercent);
  if (direction != 0 && direction != state.direction) {
    writeMotorPwm(pwmChannel, 0);
    delayMicroseconds(150);
  }
  if (direction != 0) digitalWrite(directionPin, direction > 0 ? HIGH : LOW);
  state.direction = direction;
  state.speedPercent = speed;
  writeMotorPwm(pwmChannel, speed);
}

void driveMotors(int leftPercent, int rightPercent) {
  digitalWrite(BoardPins::motorSleepN, HIGH);
  digitalWrite(BoardPins::motorDriveOff, LOW);
  delayMicroseconds(50);
  setMotor(leftMotor, BoardPins::motorLeftDir, Config::motorLeftPwmChannel,
           leftPercent);
  setMotor(rightMotor, BoardPins::motorRightDir, Config::motorRightPwmChannel,
           rightPercent);
}

void stopMotorOutputs(bool sleepDrivers = true) {
  writeMotorPwm(Config::motorLeftPwmChannel, 0);
  writeMotorPwm(Config::motorRightPwmChannel, 0);
  leftMotor = {};
  rightMotor = {};
  if (sleepDrivers) {
    digitalWrite(BoardPins::motorDriveOff, HIGH);
    digitalWrite(BoardPins::motorSleepN, LOW);
  }
}

int gruzikPwmToPercent(float pwm) {
  const float scaled = pwm / Config::gruzik4PwmMaximum *
                       Config::motorPwmLimitPercent;
  return static_cast<int>(roundf(clampValue(
      scaled, -Config::motorPwmLimitPercent, Config::motorPwmLimitPercent)));
}

void IRAM_ATTR updateQuadrature(EncoderState &encoder, uint8_t pinA,
                                uint8_t pinB) {
  static constexpr int8_t transitions[16] = {
      0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  const uint8_t current = (digitalRead(pinA) << 1) | digitalRead(pinB);
  encoder.count += transitions[((encoder.previousState << 2) | current) & 0x0F];
  encoder.previousState = current;
}

void IRAM_ATTR leftEncoderEdge() {
  updateQuadrature(leftEncoder, BoardPins::encoderLeftA, BoardPins::encoderLeftB);
}
void IRAM_ATTR rightEncoderEdge() {
  updateQuadrature(rightEncoder, BoardPins::encoderRightA,
                   BoardPins::encoderRightB);
}
void IRAM_ATTR leftEncoderIndex() { ++leftEncoder.indexPulses; }
void IRAM_ATTR rightEncoderIndex() { ++rightEncoder.indexPulses; }

void setupEncoders() {
  pinMode(BoardPins::encoderLeftA, INPUT_PULLUP);
  pinMode(BoardPins::encoderLeftB, INPUT_PULLUP);
  pinMode(BoardPins::encoderLeftIndex, INPUT_PULLUP);
  pinMode(BoardPins::encoderRightA, INPUT_PULLUP);
  pinMode(BoardPins::encoderRightB, INPUT_PULLUP);
  pinMode(BoardPins::encoderRightIndex, INPUT_PULLUP);
  leftEncoder.previousState = (digitalRead(BoardPins::encoderLeftA) << 1) |
                              digitalRead(BoardPins::encoderLeftB);
  rightEncoder.previousState = (digitalRead(BoardPins::encoderRightA) << 1) |
                               digitalRead(BoardPins::encoderRightB);
  attachInterrupt(BoardPins::encoderLeftA, leftEncoderEdge, CHANGE);
  attachInterrupt(BoardPins::encoderLeftB, leftEncoderEdge, CHANGE);
  attachInterrupt(BoardPins::encoderLeftIndex, leftEncoderIndex, RISING);
  attachInterrupt(BoardPins::encoderRightA, rightEncoderEdge, CHANGE);
  attachInterrupt(BoardPins::encoderRightB, rightEncoderEdge, CHANGE);
  attachInterrupt(BoardPins::encoderRightIndex, rightEncoderIndex, RISING);
}

void resetOdometry() {
  odometry = {};
  pendingGyroDeltaRad = 0.0f;
  noInterrupts();
  leftEncoder.count = 0;
  rightEncoder.count = 0;
  leftEncoder.indexPulses = 0;
  rightEncoder.indexPulses = 0;
  interrupts();
  leftEncoder.lastCount = 0;
  rightEncoder.lastCount = 0;
}

void updateEncodersAndOdometry() {
  const uint32_t now = millis();
  if (now - lastEncoderMs < Config::encoderPeriodMs) return;
  const float dt = (now - lastEncoderMs) * 0.001f;
  lastEncoderMs = now;

  noInterrupts();
  const int32_t leftCount = leftEncoder.count;
  const int32_t rightCount = rightEncoder.count;
  interrupts();

  const int32_t leftRawDelta = leftCount - leftEncoder.lastCount;
  const int32_t rightRawDelta = rightCount - rightEncoder.lastCount;
  leftEncoder.lastCount = leftCount;
  rightEncoder.lastCount = rightCount;
  leftEncoder.lastDelta = leftRawDelta * Config::leftEncoderDirection;
  rightEncoder.lastDelta = rightRawDelta * Config::rightEncoderDirection;

  const float leftDeltaM =
      leftEncoder.lastDelta /
      static_cast<float>(Config::encoderCountsPerWheelRevolution) *
      Config::wheelCircumferenceM;
  const float rightDeltaM =
      rightEncoder.lastDelta /
      static_cast<float>(Config::encoderCountsPerWheelRevolution) *
      Config::wheelCircumferenceM;
  leftEncoder.rpm = leftEncoder.lastDelta /
                    static_cast<float>(Config::encoderCountsPerWheelRevolution) /
                    (dt / 60.0f);
  rightEncoder.rpm = rightEncoder.lastDelta /
                     static_cast<float>(Config::encoderCountsPerWheelRevolution) /
                     (dt / 60.0f);
  leftEncoder.metersPerSecond = leftDeltaM / dt;
  rightEncoder.metersPerSecond = rightDeltaM / dt;

  const float encoderDeltaYaw = (rightDeltaM - leftDeltaM) / Config::wheelBaseM;
  float gyroDeltaYaw = pendingGyroDeltaRad;
  pendingGyroDeltaRad = 0.0f;
  if (imuMode == ImuMode::None) gyroDeltaYaw = encoderDeltaYaw;

  const float fusedDeltaYaw =
      imuMode == ImuMode::None
          ? encoderDeltaYaw
          : Config::odometryGyroWeight * gyroDeltaYaw +
                (1.0f - Config::odometryGyroWeight) * encoderDeltaYaw;
  const float deltaDistance = 0.5f * (leftDeltaM + rightDeltaM);
  const float integrationYaw = odometry.yawRad + 0.5f * fusedDeltaYaw;
  odometry.xM += deltaDistance * cosf(integrationYaw);
  odometry.yM += deltaDistance * sinf(integrationYaw);
  odometry.yawRad = normalizeAngle(odometry.yawRad + fusedDeltaYaw);
  odometry.gyroYawRad = normalizeAngle(odometry.gyroYawRad + gyroDeltaYaw);
  odometry.encoderYawRad =
      normalizeAngle(odometry.encoderYawRad + encoderDeltaYaw);
  odometry.totalDistanceM += fabsf(deltaDistance);
}

void setupLineSensors() {
  pinMode(BoardPins::lineMuxS0, OUTPUT);
  pinMode(BoardPins::lineMuxS1, OUTPUT);
  pinMode(BoardPins::lineMuxS2, OUTPUT);
  pinMode(BoardPins::lineMuxS3, OUTPUT);
  pinMode(BoardPins::lineMuxEnable, OUTPUT);
  digitalWrite(BoardPins::lineMuxEnable, LOW);
}

bool setupAdc() {
  // GPIO1 (battery divider) and GPIO10 (74HC4067 Z) are ADC1 channels 0 and
  // 9. ESP32-S3's legacy rtc-controller conversion can remain busy forever;
  // use the ADC digital/DMA controller instead. Its reads have a timeout, so
  // an ADC fault cannot stall the robot or its BLE link.
  pinMode(BoardPins::batteryAdc, ANALOG);
  pinMode(BoardPins::lineMuxAdc, ANALOG);
  const uint32_t channelMask = (1UL << ADC1_CHANNEL_0) |
                               (1UL << ADC1_CHANNEL_9);
  adc_digi_init_config_t init{};
  // BLE/LittleFS setup takes longer than a 256-byte DMA queue can hold at
  // 20 kSa/s. Keep a generous queue and only start conversion after setup()
  // has completed, otherwise the ADC driver stops on an early overflow.
  init.max_store_buf_size = 8192;
  // Eight results per DMA interrupt keep the pool draining even while BLE,
  // motor control and the 16-way mux scan are running in the same loop.
  init.conv_num_each_intr = 32;
  init.adc1_chan_mask = channelMask;
  if (adc_digi_initialize(&init) != ESP_OK) return false;

  static adc_digi_pattern_config_t pattern[2];
  // adc_digi_pattern_config_t stores the hardware unit index: ADC1 = 0.
  pattern[0] = {ADC_ATTEN_DB_12, ADC1_CHANNEL_0, 0,
                SOC_ADC_DIGI_MAX_BITWIDTH};
  pattern[1] = {ADC_ATTEN_DB_12, ADC1_CHANNEL_9, 0,
                SOC_ADC_DIGI_MAX_BITWIDTH};
  adc_digi_configuration_t config{};
  config.conv_limit_en = false;
  config.pattern_num = 2;
  config.adc_pattern = pattern;
  // 10 kSa/s yields roughly five fresh GPIO10 samples for every mux position
  // without overrunning the DMA queue on the ESP32-S3.
  config.sample_freq_hz = 10000;
  config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  if (adc_digi_controller_configure(&config) != ESP_OK) {
    adc_digi_deinitialize();
    return false;
  }
  return true;
}

bool startAdc() {
  if (adc_digi_start() != ESP_OK) {
    adc_digi_deinitialize();
    return false;
  }
  adcReady = true;
  return true;
}

bool refreshAdcSamples(uint32_t timeoutMs) {
  if (!adcReady) return false;
  uint8_t samples[128];
  uint32_t bytesRead = 0;
  const esp_err_t result =
      adc_digi_read_bytes(samples, sizeof(samples), &bytesRead, timeoutMs);
  adcLastReadResult = result;
  adcLastBytesRead = bytesRead;
  if (result != ESP_OK || bytesRead < sizeof(adc_digi_output_data_t)) {
    ++adcReadErrorCount;
    return false;
  }
  ++adcReadOkCount;
  for (uint32_t offset = 0;
       offset + sizeof(adc_digi_output_data_t) <= bytesRead;
       offset += sizeof(adc_digi_output_data_t)) {
    const auto *sample = reinterpret_cast<const adc_digi_output_data_t *>(
        samples + offset);
    // TYPE2 encodes ADC1 as unit 0 and carries its channel in four bits.
    if (sample->type2.unit == 0 && sample->type2.channel < 10) {
      const uint8_t channel = sample->type2.channel;
      adcLatest[channel] = sample->type2.data;
      adcSeenMask |= static_cast<uint16_t>(1U << channel);
      ++adcParsedSampleCount;
    }
  }
  return adcSeenMask != 0;
}

bool readAdc(uint8_t pin, uint16_t &value) {
  const uint8_t channel = pin == BoardPins::batteryAdc ? ADC1_CHANNEL_0
                                                         : ADC1_CHANNEL_9;
  refreshAdcSamples(2);
  if ((adcSeenMask & (1U << channel)) == 0) return false;
  value = adcLatest[channel];
  return true;
}

void selectLineChannel(uint8_t channel) {
  digitalWrite(BoardPins::lineMuxS0, channel & 0x01);
  digitalWrite(BoardPins::lineMuxS1, channel & 0x02);
  digitalWrite(BoardPins::lineMuxS2, channel & 0x04);
  digitalWrite(BoardPins::lineMuxS3, channel & 0x08);
}

void scanLineSensors() {
  const uint32_t now = millis();
  if (now - lastLineScanMs < Config::lineScanPeriodMs) return;
  lastLineScanMs = now;
  // Read one mux position per call. A complete 16-channel sweep takes 16 ms,
  // while BLE, motor control and watchdog servicing continue between samples.
  const uint8_t channel = nextLineMuxChannel;
  selectLineChannel(channel);
  delayMicroseconds(250);
  uint16_t raw = 0;
  if (readAdc(BoardPins::lineMuxAdc, raw)) {
    lineRaw[Config::lineSensorByMuxChannel[channel]] = raw;
  }
  nextLineMuxChannel = (channel + 1) & 0x0F;
}

void updateBattery() {
  if (!adcReady) {
    batteryVoltage = 0.0f;
    batteryPercent = 0.0f;
    return;
  }
  const uint32_t now = millis();
  if (now - lastBatteryMs < Config::batteryPeriodMs) return;
  lastBatteryMs = now;
  uint32_t rawSum = 0;
  for (uint8_t sample = 0; sample < 4; ++sample) {
    uint16_t raw = 0;
    if (readAdc(BoardPins::batteryAdc, raw)) rawSum += raw;
  }
  const float adcVoltage = (rawSum / 4.0f) * (3.3f / 4095.0f);
  batteryVoltage = adcVoltage * Config::batteryDivider;
  batteryPercent = clampValue(
      100.0f * (batteryVoltage - Config::batteryEmptyVoltage) /
          (Config::batteryFullVoltage - Config::batteryEmptyVoltage),
      0.0f, 100.0f);
}

void initializeImus() {
  const uint8_t u7Status = imuU7.begin();
  const uint8_t u10Status = imuU10.begin();
  if (u7Status == 1) imuU7.calibrateGyroZ(300, 1);
  if (u10Status == 1) imuU10.calibrateGyroZ(300, 1);

  const uint8_t count = (imuU7.initialized() ? 1 : 0) +
                        (imuU10.initialized() ? 1 : 0);
  imuMode = count == 2 ? ImuMode::Dual
                       : (count == 1 ? ImuMode::Single : ImuMode::None);
  lastImuUs = micros();
  Serial.printf(
      "ICM-42688-P: mode=%u U7(status=%u who=0x%02X) "
      "U10(status=%u who=0x%02X)\n",
      count, u7Status, imuU7.whoAmI(), u10Status, imuU10.whoAmI());
}

void updateStatusLed(uint32_t nowMs) {
  // GRUZIK4.0 has two LEDs: LED1 toggles every 500 ms and LED2 is toggled on
  // a successful Start. This PCB has one controllable LED (LED3), so it
  // preserves the useful meaning: blink at the GRUZIK4.0 LED1 cadence while
  // stopped; stay lit for every active drive mode after Start.
  const bool robotRunning = robotMode != RobotMode::Stopped;
  const bool shouldBeOn = robotRunning ||
                          ((nowMs / Config::statusLedStoppedToggleMs) % 2u ==
                           0u);
  if (shouldBeOn == statusLedOn) return;
  statusLedOn = shouldBeOn;
  digitalWrite(BoardPins::statusLed, statusLedOn ? HIGH : LOW);
}

void diagnoseImuMisoIdleLevel() {
  // A missing SPI device should leave MISO high when this weak pull-up is
  // enabled. A low level here points to a solder bridge, a powered-down IC
  // clamping its pin, or the MISO net being held low externally.
  SPI.end();
  digitalWrite(BoardPins::imuU7Cs, HIGH);
  digitalWrite(BoardPins::imuU10Cs, HIGH);
  pinMode(BoardPins::imuMiso, INPUT_PULLUP);
  delay(2);
  const int misoIdle = digitalRead(BoardPins::imuMiso);
  Serial.printf("ICM diagnostic: MISO idle with ESP pull-up = %d\n", misoIdle);
  SPI.begin(BoardPins::imuSclk, BoardPins::imuMiso, BoardPins::imuMosi,
            BoardPins::imuU7Cs);
}

void updateImus() {
  const uint32_t nowUs = micros();
  if (nowUs - lastImuUs < Config::imuPeriodUs) return;
  const float dt = (nowUs - lastImuUs) * 0.000001f;
  lastImuUs = nowUs;

  const bool u7Ok = imuU7.initialized() && imuU7.readGyroZ();
  const bool u10Ok = imuU10.initialized() && imuU10.readGyroZ();
  const uint8_t validCount = (u7Ok ? 1 : 0) + (u10Ok ? 1 : 0);
  if (validCount == 0) {
    gyroZRawDps = 0.0f;
    gyroZBiasDps = 0.0f;
    gyroZDps = 0.0f;
    return;
  }

  gyroZRawDps = 0.0f;
  gyroZBiasDps = 0.0f;
  gyroZDps = 0.0f;
  if (u7Ok) {
    gyroZRawDps += imuU7.gyroZRawDps() * imuU7.orientationSign();
    gyroZBiasDps += imuU7.gyroZBiasDps() * imuU7.orientationSign();
    gyroZDps += imuU7.gyroZDps();
  }
  if (u10Ok) {
    gyroZRawDps += imuU10.gyroZRawDps() * imuU10.orientationSign();
    gyroZBiasDps += imuU10.gyroZBiasDps() * imuU10.orientationSign();
    gyroZDps += imuU10.gyroZDps();
  }
  gyroZRawDps /= validCount;
  gyroZBiasDps /= validCount;
  gyroZDps /= validCount;
  pendingGyroDeltaRad += gyroZDps * DEG_TO_RAD * dt;

  const bool stationary =
      robotMode == RobotMode::Stopped && leftEncoder.lastDelta == 0 &&
      rightEncoder.lastDelta == 0 &&
      fabsf(gyroZDps) < Config::gyroStationaryDps;
  if (stationary) {
    if (u7Ok) imuU7.updateGyroZBias(Config::gyroBiasLearnRate);
    if (u10Ok) imuU10.updateGyroZBias(Config::gyroBiasLearnRate);
  }

  static uint16_t temperatureDivider = 0;
  if (++temperatureDivider >= 1000) {
    temperatureDivider = 0;
    float sum = 0.0f;
    uint8_t count = 0;
    if (imuU7.initialized() && imuU7.readTemperature()) {
      sum += imuU7.temperatureC();
      ++count;
    }
    if (imuU10.initialized() && imuU10.readTemperature()) {
      sum += imuU10.temperatureC();
      ++count;
    }
    if (count > 0) gyroTemperatureC = sum / count;
  }
}

String imuStatusLine() {
  if (!Config::imuEnabled) {
    return "IMU_MODE,0,U7=0,U10=0,DISABLED=1\r\n";
  }
  return "IMU_MODE," + String(static_cast<uint8_t>(imuMode)) + ",U7=" +
         String(imuU7.initialized() ? 1 : 0) + ",U10=" +
         String(imuU10.initialized() ? 1 : 0) + ",WHO7=0x" +
         String(imuU7.whoAmI(), HEX) + ",WHO10=0x" +
         String(imuU10.whoAmI(), HEX) + "\r\n";
}

void sendText(const String &text) {
  if (!bleClientConnected || bleTextCharacteristic == nullptr) return;
  constexpr size_t chunkSize = 150;
  for (size_t offset = 0; offset < text.length(); offset += chunkSize) {
    const size_t count = min(chunkSize, text.length() - offset);
    bleTextCharacteristic->setValue(
        reinterpret_cast<const uint8_t *>(text.c_str() + offset), count);
    bleTextCharacteristic->notify();
    delay(4);
  }
}

void closeMapFiles() {
  if (recordingFile) recordingFile.close();
  if (uploadFile) uploadFile.close();
}

void stopRobot() {
  closeMapFiles();
  lineFollower.stop();
  stopMotorOutputs(true);
  robotMode = RobotMode::Stopped;
  playbackFinishAtMs = 0;
}

void sendStartStatus(const char *kind) {
  sendText("STARTING," + String(kind) + "\r\nStart\r\nBattery = " +
           String(batteryVoltage, 2) + " V\r\nSpeed_level = 1.00\r\n" +
           imuStatusLine());
}

void startNormal() {
  closeMapFiles();
  resetOdometry();
  lineFollower.start();
  robotMode = RobotMode::Normal;
  playbackFinishAtMs = 0;
  sendStartStatus("normal");
}

void writeRecordedPoint(float x, float y, float sensorX, float sensorY,
                        float speedMps, float errorP, float errorD) {
  if (!recordingFile) return;
  recordingFile.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", x,
                       y, sensorX, sensorY, speedMps, errorP, errorD);
  recordingFile.flush();
  ++recordedPointCount;
}

void startMapping() {
  closeMapFiles();
  LittleFS.remove(Config::recordedMapPath);
  recordingFile = LittleFS.open(Config::recordedMapPath, FILE_WRITE);
  if (!recordingFile) {
    sendText("MAP_ERROR,open_write,GRUZIK.txt,1\r\n");
    return;
  }
  resetOdometry();
  lineFollower.start();
  robotMode = RobotMode::Mapping;
  recordedPointCount = 0;
  recordedDistanceM = 0.0f;
  mapStartX = odometry.xM;
  mapStartY = odometry.yM;
  mapStartSensorX = mapStartX + Config::sensorOffsetM;
  mapStartSensorY = mapStartY;
  lastMapSensorX = mapStartSensorX;
  lastMapSensorY = mapStartSensorY;
  previousMapLineError = 0.0f;
  lastMapRecordMs = 0;
  writeRecordedPoint(mapStartX, mapStartY, mapStartSensorX, mapStartSensorY,
                     0.0f, 0.0f, 0.0f);
  sendStartStatus("mapping");
}

bool loadPlaybackMap() {
  playbackMap.clear();
  File file = LittleFS.open(Config::optimizedMapPath, FILE_READ);
  if (!file) return false;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;
    MapPoint point;
    const int parsed =
        sscanf(line.c_str(), "%f%f%f", &point.x, &point.y, &point.speed);
    if (parsed >= 2) {
      if (parsed < 3) point.speed = mapDefaultSpeed;
      point.speed = clampValue(point.speed, -Config::gruzik4PwmMaximum,
                               Config::gruzik4PwmMaximum);
      playbackMap.push_back(point);
    }
  }
  file.close();
  return !playbackMap.empty();
}

void startPlayback() {
  closeMapFiles();
  if (!loadPlaybackMap()) {
    sendText("MAP_ERROR,open_playback,map.txt,4\r\n");
    return;
  }
  resetOdometry();
  lineFollower.start();
  robotMode = RobotMode::Playback;
  playbackTargetIndex = 0;
  mapErrorSum = 0.0f;
  mapLastError = 0.0f;
  mapFilteredDerivative = 0.0f;
  playbackFinishAtMs = 0;
  sendStartStatus("playback");
}

void updateMapping() {
  if (robotMode != RobotMode::Mapping || !recordingFile) return;
  const uint32_t now = millis();
  if (now - lastMapRecordMs < Config::mapRecordPeriodMs) return;
  lastMapRecordMs = now;

  const float sensorX = odometry.xM +
                        Config::sensorOffsetM * cosf(odometry.yawRad);
  const float sensorY = odometry.yM +
                        Config::sensorOffsetM * sinf(odometry.yawRad);
  const float dx = sensorX - lastMapSensorX;
  const float dy = sensorY - lastMapSensorY;
  recordedDistanceM += sqrtf(dx * dx + dy * dy);
  lastMapSensorX = sensorX;
  lastMapSensorY = sensorY;

  const float speed = 0.5f * (leftEncoder.metersPerSecond +
                              rightEncoder.metersPerSecond);
  const float error = lineFollower.error();
  const float derivative = error - previousMapLineError;
  previousMapLineError = error;
  writeRecordedPoint(odometry.xM, odometry.yM, sensorX, sensorY, speed, error,
                     derivative);

  const float closeDx = odometry.xM - mapStartX;
  const float closeDy = odometry.yM - mapStartY;
  const float closeDistance = sqrtf(closeDx * closeDx + closeDy * closeDy);
  if (recordedDistanceM >= Config::mapCloseMinimumDistanceM &&
      recordedPointCount >= Config::mapCloseMinimumPoints &&
      closeDistance <= Config::mapCloseRadiusM) {
    writeRecordedPoint(mapStartX, mapStartY, mapStartSensorX, mapStartSensorY,
                       speed, error, derivative);
    recordingFile.close();
    lineFollower.stop();
    stopMotorOutputs(true);
    robotMode = RobotMode::Stopped;
    sendText("MAP_AUTO_CLOSED," + String(recordedPointCount) + "\r\n");
  }
}

bool shouldAdvanceMapTarget(float distanceToTarget) {
  if (distanceToTarget < Config::mapTargetRadiusM) return true;
  if (playbackTargetIndex == 0 || playbackTargetIndex >= playbackMap.size()) {
    return false;
  }
  const MapPoint &previous = playbackMap[playbackTargetIndex - 1];
  const MapPoint &target = playbackMap[playbackTargetIndex];
  const float segmentX = target.x - previous.x;
  const float segmentY = target.y - previous.y;
  const float segmentLengthSquared = segmentX * segmentX + segmentY * segmentY;
  if (segmentLengthSquared < 0.000001f) return true;
  const float poseX = odometry.xM - previous.x;
  const float poseY = odometry.yM - previous.y;
  const float projection =
      (poseX * segmentX + poseY * segmentY) / segmentLengthSquared;
  const float lateralError =
      fabsf(poseX * segmentY - poseY * segmentX) /
      sqrtf(segmentLengthSquared);
  return (projection > 1.02f && lateralError < Config::mapAdvanceLateralM) ||
         projection > 1.30f;
}

void finishPlayback() {
  robotMode = RobotMode::Normal;
  lineFollower.start();
  playbackFinishAtMs = millis() + Config::mapFinishPidMs;
  sendText("MAP_PLAYBACK_DONE\r\n");
}

void updatePlayback() {
  if (robotMode != RobotMode::Playback || playbackMap.empty()) return;

  LineFollowerOutput lineOutput = lineFollower.update(lineRaw, millis());
  while (playbackTargetIndex < playbackMap.size()) {
    const MapPoint &target = playbackMap[playbackTargetIndex];
    const float dx = target.x - odometry.xM;
    const float dy = target.y - odometry.yM;
    if (!shouldAdvanceMapTarget(sqrtf(dx * dx + dy * dy))) break;
    ++playbackTargetIndex;
  }
  if (playbackTargetIndex >= playbackMap.size()) {
    finishPlayback();
    return;
  }

  const MapPoint &target = playbackMap[playbackTargetIndex];
  float previousX = 0.0f;
  float previousY = 0.0f;
  if (playbackTargetIndex > 0) {
    previousX = playbackMap[playbackTargetIndex - 1].x;
    previousY = playbackMap[playbackTargetIndex - 1].y;
  }
  const float segmentX = target.x - previousX;
  const float segmentY = target.y - previousY;
  const float segmentLength = sqrtf(segmentX * segmentX + segmentY * segmentY);
  float error = 0.0f;
  if (segmentLength > 0.000001f) {
    const float targetYaw = atan2f(segmentY, segmentX);
    const float poseX = odometry.xM - previousX;
    const float poseY = odometry.yM - previousY;
    float crossTrack =
        (poseX * segmentY - poseY * segmentX) / segmentLength;
    crossTrack = clampValue(crossTrack, -Config::playbackCrosstrackLimitM,
                            Config::playbackCrosstrackLimitM);
    const float speedRatio =
        clampValue(fabsf(target.speed) / Config::gruzik4PwmMaximum, 0.0f, 1.0f);
    const float lookahead = Config::playbackLookaheadBaseM +
                            Config::playbackLookaheadSpeedM * speedRatio;
    error = normalizeAngle((targetYaw - odometry.yawRad) +
                           atan2f(crossTrack, lookahead));
  } else {
    error = normalizeAngle(
        atan2f(target.y - odometry.yM, target.x - odometry.xM) -
        odometry.yawRad);
  }

  mapErrorSum = clampValue(mapErrorSum + error, -3.0f, 3.0f);
  const float derivative = normalizeAngle(error - mapLastError);
  mapLastError = error;
  mapFilteredDerivative += Config::playbackDerivativeAlpha *
                           (derivative - mapFilteredDerivative);
  float odomCorrection =
      mapP * error + mapI * mapErrorSum + mapD * mapFilteredDerivative;
  const float odomLimit = clampValue(fabsf(target.speed) * 0.85f, 45.0f,
                                     Config::playbackOdomCorrectionLimit);
  odomCorrection = clampValue(odomCorrection, -odomLimit, odomLimit);

  float lineCorrection = 0.0f;
  if (lineOutput.valid) {
    lineCorrection = 0.5f * (lineOutput.left - lineOutput.right);
    lineCorrection = clampValue(lineCorrection,
                                -Config::playbackLineCorrectionLimit,
                                Config::playbackLineCorrectionLimit);
  }
  float confidence = 0.0f;
  if (lineFollower.activeCount() > 0) {
    const float centerError = fabsf(lineFollower.error());
    confidence = 1.0f -
                 clampValue((centerError - 1800.0f) / 3600.0f, 0.0f, 1.0f);
    if (lineFollower.activeCount() > 7) confidence *= 0.45f;
  }

  float motorCorrection = -odomCorrection +
                          lineCorrection * Config::playbackLineWeight * confidence;
  motorCorrection = clampValue(motorCorrection,
                               -Config::playbackCorrectionLimit,
                               Config::playbackCorrectionLimit);
  float baseSpeed = target.speed;
  baseSpeed *= 1.0f - clampValue(fabsf(error), 0.0f, 1.0f) *
                          Config::playbackTurnSpeedReduction;
  if (lineFollower.activeCount() == 0) {
    baseSpeed *= Config::playbackLostLineSpeedMultiplier;
  }
  const float left = clampValue(baseSpeed + motorCorrection,
                                -Config::gruzik4PwmMaximum,
                                Config::gruzik4PwmMaximum);
  const float right = clampValue(baseSpeed - motorCorrection,
                                 -Config::gruzik4PwmMaximum,
                                 Config::gruzik4PwmMaximum);
  driveMotors(gruzikPwmToPercent(left), gruzikPwmToPercent(right));
}

void updateRobotControl() {
  if (robotMode == RobotMode::Playback) {
    updatePlayback();
    return;
  }
  if (robotMode != RobotMode::Normal && robotMode != RobotMode::Mapping) return;
  const LineFollowerOutput output = lineFollower.update(lineRaw, millis());
  if (output.valid) {
    driveMotors(gruzikPwmToPercent(output.left),
                gruzikPwmToPercent(output.right));
  }
  if (playbackFinishAtMs != 0 &&
      static_cast<int32_t>(millis() - playbackFinishAtMs) >= 0) {
    stopRobot();
    sendText("Stop\r\nBattery = " + String(batteryVoltage, 2) + " V\r\n");
  }
}

void dumpMap(const String &kind) {
  if (robotMode != RobotMode::Stopped) {
    sendText("MAP_ERROR,stop_robot_first\r\n");
    return;
  }
  const bool optimized = kind.equalsIgnoreCase("optimized");
  const char *path = optimized ? Config::optimizedMapPath : Config::recordedMapPath;
  File file = LittleFS.open(path, FILE_READ);
  if (!file) {
    sendText("MAP_ERROR,open_read," + String(optimized ? "map.txt" : "GRUZIK.txt") +
             ",4\r\n");
    return;
  }
  const String dumpKind = optimized ? "optimized" : "recorded";
  sendText("MAP_BEGIN," + dumpKind + "\r\n");
  uint16_t count = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;
    sendText("MAP," + line + "\r\n");
    ++count;
  }
  file.close();
  sendText("MAP_END," + String(count) + "\r\n");
}

void beginMapUpload(uint16_t expectedCount) {
  if (robotMode != RobotMode::Stopped) {
    sendText("UPLOAD_ERROR,stop_robot_first\r\n");
    return;
  }
  if (uploadFile) uploadFile.close();
  LittleFS.remove(Config::optimizedMapPath);
  uploadFile = LittleFS.open(Config::optimizedMapPath, FILE_WRITE);
  if (!uploadFile) {
    sendText("UPLOAD_ERROR,open_write,map.txt,1\r\n");
    return;
  }
  uploadedPointCount = 0;
  expectedUploadPointCount = expectedCount;
  sendText("UPLOAD_READY," + String(expectedCount) + "\r\n");
}

void uploadMapPoint(const String &value) {
  if (!uploadFile) {
    sendText("UPLOAD_ERROR,no_file\r\n");
    return;
  }
  MapPoint point;
  if (sscanf(value.c_str(), "%f,%f,%f", &point.x, &point.y, &point.speed) < 2) {
    sendText("UPLOAD_ERROR,bad_point\r\n");
    return;
  }
  uploadFile.printf("%.3f\t%.3f\t%.3f\n", point.x, point.y, point.speed);
  ++uploadedPointCount;
  if (uploadedPointCount % 50 == 0) {
    sendText("UPLOAD_PROGRESS," + String(uploadedPointCount) + ',' +
             String(expectedUploadPointCount) + "\r\n");
  }
}

void finishMapUpload() {
  if (uploadFile) {
    uploadFile.flush();
    uploadFile.close();
  }
  sendText("UPLOAD_DONE," + String(uploadedPointCount) + ',' +
           String(expectedUploadPointCount) + "\r\n");
}

void selectRobotMode(SelectedMode mode) {
  if (robotMode != RobotMode::Stopped) {
    sendText("Stop robot before changing state\r\n");
    return;
  }
  selectedMode = mode;
  if (mode == SelectedMode::Normal) sendText("State: PID\r\n");
  if (mode == SelectedMode::Mapping) sendText("State: Mapping\r\n");
  if (mode == SelectedMode::Playback) sendText("State: UnMapping\r\n");
}

void startSelectedMode() {
  if (selectedMode == SelectedMode::Normal) startNormal();
  if (selectedMode == SelectedMode::Mapping) startMapping();
  if (selectedMode == SelectedMode::Playback) startPlayback();
}

void processCommandLine(String command) {
  command.trim();
  if (command.isEmpty()) return;
  const int separator = command.indexOf('=');
  const String key = separator >= 0 ? command.substring(0, separator) : command;
  const String value = separator >= 0 ? command.substring(separator + 1) : "";

  if (key == "Kp") lineFollower.kp = value.toFloat();
  else if (key == "Kd") lineFollower.kd = value.toFloat();
  else if (key == "Base_speed") lineFollower.baseSpeed = value.toFloat();
  else if (key == "Max_speed") lineFollower.maxSpeed = value.toFloat();
  else if (key == "Sharp_bend_speed_right") lineFollower.sharpRight = value.toFloat();
  else if (key == "Sharp_bend_speed_left") lineFollower.sharpLeft = value.toFloat();
  else if (key == "Bend_speed_right") lineFollower.bendRight = value.toFloat();
  else if (key == "Bend_speed_left") lineFollower.bendLeft = value.toFloat();
  else if (key == "Treshold") lineFollower.threshold = constrain(value.toInt(), 0, 4095);
  else if (key == "Turbine_Speed") turbineSpeed = value.toFloat();
  else if (key == "Turbine_Prep_Time") turbinePreparationTime = value.toFloat();
  else if (key == "MapP") mapP = value.toFloat();
  else if (key == "MapI") mapI = value.toFloat();
  else if (key == "MapD") mapD = value.toFloat();
  else if (key == "MapSpeed") mapDefaultSpeed = value.toFloat();
  else if (key == "StartNormal") startNormal();
  else if (key == "StartMapping") startMapping();
  else if (key == "StartPlayback") startPlayback();
  else if (key == "Mode") {
    const char mode = value.length() ? value[0] : 'N';
    if (mode == 'N') {
      stopRobot();
      sendText("Stop\r\nBattery = " + String(batteryVoltage, 2) + " V\r\n");
    } else if (mode == 'Y' || mode == 'C') {
      startSelectedMode();
    } else if (mode == 'P') {
      selectRobotMode(SelectedMode::Normal);
    } else if (mode == 'M') {
      selectRobotMode(SelectedMode::Mapping);
    } else if (mode == 'U') {
      selectRobotMode(SelectedMode::Playback);
    }
  } else if (key == "State") {
    if (value == "PID" || value == "Pid" || value == "P")
      selectRobotMode(SelectedMode::Normal);
    else if (value == "Mapping" || value == "M")
      selectRobotMode(SelectedMode::Mapping);
    else if (value == "UnMapping" || value == "U")
      selectRobotMode(SelectedMode::Playback);
  } else if (key == "Manual" || key == "Joystick") {
    lineFollower.stop();
    robotMode = RobotMode::Manual;
    const int comma = value.indexOf(',');
    if (comma >= 0) {
      const float left = value.substring(0, comma).toFloat();
      const float right = value.substring(comma + 1).toFloat();
      if (fabsf(left) < 0.5f && fabsf(right) < 0.5f) {
        stopMotorOutputs(true);
        robotMode = RobotMode::Stopped;
      } else {
        driveMotors(gruzikPwmToPercent(left), gruzikPwmToPercent(right));
      }
    }
  } else if (key == "CleanSpeed" || key == "TireCleanSpeed") {
    cleanSpeed = value.toFloat();
  } else if (key == "Clean" || key == "TireClean") {
    lineFollower.stop();
    if (value.toInt() > 0) {
      robotMode = RobotMode::Cleaning;
      driveMotors(gruzikPwmToPercent(cleanSpeed),
                  gruzikPwmToPercent(cleanSpeed));
      sendText("CLEAN,start\r\n");
    } else {
      stopMotorOutputs(true);
      robotMode = RobotMode::Stopped;
      sendText("CLEAN,stop\r\n");
    }
  } else if (key == "Telemetry" || key == "Debug") {
    if (value.equalsIgnoreCase("odom") || value == "1") {
      telemetryMode = TelemetryMode::Odom;
      sendText("TELEMETRY,odom\r\n" + imuStatusLine());
    } else if (value.equalsIgnoreCase("debug") || value == "2") {
      telemetryMode = TelemetryMode::Debug;
      sendText("TELEMETRY,debug\r\n" + imuStatusLine());
    } else {
      telemetryMode = TelemetryMode::Off;
      sendText("TELEMETRY,off\r\n");
    }
  } else if (key == "MapDump") {
    dumpMap(value);
  } else if (key == "MapUploadBegin") {
    beginMapUpload(constrain(value.toInt(), 0, 65535));
  } else if (key == "MapPoint") {
    uploadMapPoint(value);
  } else if (key == "MapUploadEnd") {
    finishMapUpload();
  } else if (key == "Add_Simple_Map_Point") {
    File file = LittleFS.open(Config::optimizedMapPath, FILE_APPEND);
    if (file) {
      file.printf("%.3f\t%.3f\t%.3f\n", odometry.xM, odometry.yM,
                  mapDefaultSpeed);
      file.close();
    }
  } else if (key == "BtName" || key == "BTName" ||
             key == "BluetoothName" || key == "BtNameNow" ||
             key == "BTNameNow" || key == "BluetoothNameNow") {
    sendText("BT_NAME_OK,gruzik 5.0,0\r\n");
  } else if (key == "ImuStatus") {
    sendText(imuStatusLine());
  }
}

void processBlePayload(const std::string &payload) {
  bleReceiveBuffer += String(payload.c_str());
  int newline = -1;
  while ((newline = bleReceiveBuffer.indexOf('\n')) >= 0) {
    String command = bleReceiveBuffer.substring(0, newline);
    bleReceiveBuffer.remove(0, newline + 1);
    processCommandLine(command);
  }
  // BLE writes from simple clients may contain one complete command without
  // a newline. All GRUZIK4.0 commands contain '=' and fit in one GATT write.
  if (!bleReceiveBuffer.isEmpty() && bleReceiveBuffer.indexOf('=') >= 0 &&
      bleReceiveBuffer.length() < 180) {
    String command = bleReceiveBuffer;
    bleReceiveBuffer = "";
    processCommandLine(command);
  }
}

void sendTelemetry() {
  if (!bleClientConnected || telemetryMode == TelemetryMode::Off) return;
  const uint32_t now = millis();
  if (now - lastTelemetryMs < Config::telemetryPeriodMs) return;
  lastTelemetryMs = now;

  const float yawDeg = odometry.yawRad * RAD_TO_DEG;
  String odomLine =
      "ODOM," + String(odometry.xM, 4) + ',' + String(odometry.yM, 4) + ',' +
      String(yawDeg, 2) + ',' + String(odometry.totalDistanceM, 4) + ',' +
      String(leftEncoder.metersPerSecond, 3) + ',' +
      String(rightEncoder.metersPerSecond, 3) + ',' + String(gyroZDps, 2) + ',' +
      String(odometry.encoderYawRad * RAD_TO_DEG, 2) + ',' +
      String(odometry.gyroYawRad * RAD_TO_DEG, 2) + "\r\n";
  sendText(odomLine);
  if (now - lastBatteryTelemetryMs >= Config::batteryTelemetryPeriodMs) {
    lastBatteryTelemetryMs = now;
    sendText("Battery = " + String(batteryVoltage, 2) + " V\r\n");
  }
  if (telemetryMode != TelemetryMode::Debug) return;

  const float leftDistance =
      leftEncoder.count /
      static_cast<float>(Config::encoderCountsPerWheelRevolution) *
      Config::wheelCircumferenceM * Config::leftEncoderDirection;
  const float rightDistance =
      rightEncoder.count /
      static_cast<float>(Config::encoderCountsPerWheelRevolution) *
      Config::wheelCircumferenceM * Config::rightEncoderDirection;
  String debugLine =
      "DBG," + String(lineFollower.position(), 0) + ',' +
      String(lineFollower.activeCount()) + ',' + String(lineFollower.lastEnd()) +
      ',' + String(leftEncoder.lastDelta) + ',' +
      String(rightEncoder.lastDelta) + ',' + String(leftEncoder.rpm, 1) + ',' +
      String(rightEncoder.rpm, 1) + ',' + String(leftDistance, 4) + ',' +
      String(rightDistance, 4) + ',' + String(gyroZRawDps, 2) + ',' +
      String(gyroZBiasDps, 2) + ',' + String(gyroZDps, 2);
  // GRUZIK5.0 has the full 16-channel Sensorx16 daughterboard.
  for (uint8_t sensor = 0; sensor < 16; ++sensor) {
    debugLine += ',' + String(lineRaw[sensor]);
  }
  debugLine += "\r\n";
  sendText(debugLine);
}

class BleServerCallbacks final : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *) override {
    bleClientConnected = true;
    bleHelloPending = true;
    bleConnectedAtMs = millis();
  }

  void onDisconnect(NimBLEServer *) override {
    bleClientConnected = false;
    bleHelloPending = false;
    telemetryMode = TelemetryMode::Off;
    // Deliberately do not alter motors or line-following state on link loss.
    NimBLEDevice::startAdvertising();
  }
};

class BleCommandCallbacks final : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic) override {
    processBlePayload(characteristic->getValue());
  }
};

void setupBluetooth() {
  NimBLEDevice::init(Config::bleName);
  NimBLEDevice::setMTU(185);
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new BleServerCallbacks());
  NimBLEService *service = server->createService(Config::bleServiceUuid);
  NimBLECharacteristic *command = service->createCharacteristic(
      Config::bleCommandUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  command->setCallbacks(new BleCommandCallbacks());
  bleTextCharacteristic = service->createCharacteristic(
      Config::bleTextUuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  service->start();
  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(Config::bleServiceUuid);
  advertising->setScanResponse(true);
  advertising->start();
}

void setup() {
  Serial.begin(115200);
  delay(150);
  pinMode(BoardPins::statusLed, OUTPUT);
  digitalWrite(BoardPins::statusLed, HIGH);
  pinMode(BoardPins::imuU7Cs, OUTPUT);
  pinMode(BoardPins::imuU10Cs, OUTPUT);
  digitalWrite(BoardPins::imuU7Cs, HIGH);
  digitalWrite(BoardPins::imuU10Cs, HIGH);
  pinMode(BoardPins::imuU7Int, INPUT);
  pinMode(BoardPins::imuU10Int, INPUT);
  SPI.begin(BoardPins::imuSclk, BoardPins::imuMiso, BoardPins::imuMosi,
            BoardPins::imuU7Cs);

  setupMotors();
  setupEncoders();
  setupLineSensors();
  const bool adcConfigured = setupAdc();
  if (!adcConfigured) Serial.println("ADC DMA initialization failed");
  LittleFS.begin(true);
  if (Config::imuEnabled) {
    diagnoseImuMisoIdleLevel();
    initializeImus();
  }
  setupBluetooth();
  if (adcConfigured && !startAdc()) {
    Serial.println("ADC DMA start failed");
  }
  Serial.println("gruzik 5.0 ready (BLE, ADC, motors; no Wi-Fi)");
}

void loop() {
  if (Config::imuEnabled) updateImus();
  scanLineSensors();
  updateBattery();
  updateEncodersAndOdometry();
  updateRobotControl();
  updateMapping();
  sendTelemetry();

  const uint32_t now = millis();
  updateStatusLed(now);
  if (bleHelloPending && now - bleConnectedAtMs >= 700) {
    bleHelloPending = false;
    sendText("GRUZIK5,ready\r\n" + imuStatusLine() + "Battery = " +
             String(batteryVoltage, 2) + " V\r\n");
  }
  if (now - lastSerialStatusMs >= 2000) {
    lastSerialStatusMs = now;
    Serial.printf("GRUZIK5 adc=%u mask=0x%03X bytes=%lu read=%s ok=%lu err=%lu parsed=%lu battery=%.2fV ble=%u mode=%u imu=%s\n",
                  adcReady, adcSeenMask,
                  static_cast<unsigned long>(adcLastBytesRead),
                  esp_err_to_name(adcLastReadResult),
                  static_cast<unsigned long>(adcReadOkCount),
                  static_cast<unsigned long>(adcReadErrorCount),
                  static_cast<unsigned long>(adcParsedSampleCount), batteryVoltage, bleClientConnected,
                  static_cast<uint8_t>(robotMode),
                  Config::imuEnabled ? "on" : "off");
    Serial.print("LINE");
    for (uint8_t sensor = 0; sensor < 16; ++sensor) {
      Serial.printf(" S%02u=%u", sensor, lineRaw[sensor]);
    }
    Serial.println();
  }
  delay(1);
}
