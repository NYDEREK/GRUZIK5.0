#pragma once

#include <Arduino.h>
#include <math.h>

struct LineFollowerOutput {
  bool valid = false;
  float left = 0.0f;
  float right = 0.0f;
};

class LineFollowerController {
 public:
  float kp = 0.015f;
  float kd = 0.55f;
  float baseSpeed = 125.0f;
  float maxSpeed = 200.0f;
  float sharpRight = -75.0f;
  float sharpLeft = 120.0f;
  float bendRight = -75.0f;
  float bendLeft = 120.0f;
  uint16_t threshold = 3300;
  // Same gap behavior as GRUZIK4.0: keep the last valid motor command for
  // four seconds, then continue searching according to Last_end.
  uint32_t gapBridgeMs = 4000;

  void start() {
    running_ = true;
    lastError_ = 0.0f;
    lostSinceMs_ = 0;
    gapArmed_ = false;
    gapActive_ = false;
  }

  void stop() {
    running_ = false;
    gapActive_ = false;
    gapArmed_ = false;
  }

  bool running() const { return running_; }
  float position() const { return position_; }
  float error() const { return error_; }
  uint8_t activeCount() const { return activeCount_; }
  uint8_t lastEnd() const { return lastEnd_; }
  float lastLeft() const { return lastLeft_; }
  float lastRight() const { return lastRight_; }

  LineFollowerOutput update(const uint16_t raw[16], uint32_t nowMs) {
    LineFollowerOutput output;
    if (!running_) return output;

    float weighted = 0.0f;
    uint8_t active = 0;
    bool edgeActive = false;
    // The daughterboard is laid out SENSOR15 ... SENSOR0 from left to right.
    for (uint8_t physicalIndex = 0; physicalIndex < 16; ++physicalIndex) {
      const uint8_t channel = 15 - physicalIndex;
      const bool seesBlackLine = raw[channel] >= threshold;
      if (!seesBlackLine) continue;
      // Preserve GRUZIK4.0 steering convention: the physical left edge has
      // the highest position value and the physical right edge the lowest.
      const float weight = 16000.0f - physicalIndex * 1000.0f;
      weighted += weight;
      ++active;
      if (physicalIndex <= 1) {
        lastEnd_ = 0;
        edgeActive = true;
      } else if (physicalIndex >= 14) {
        lastEnd_ = 1;
        edgeActive = true;
      }
    }

    activeCount_ = active;
    if (active > 0) {
      position_ = weighted / active;
      error_ = 8500.0f - position_;
      lostSinceMs_ = 0;
      gapActive_ = false;

      const float derivative = error_ - lastError_;
      lastError_ = error_;
      const float correction = kp * error_ + kd * derivative;
      output.left = constrain(baseSpeed + correction, -286.0f, maxSpeed);
      output.right = constrain(baseSpeed - correction, -286.0f, maxSpeed);
      output.valid = true;
      lastLeft_ = output.left;
      lastRight_ = output.right;
      gapArmed_ = fabsf(error_) < 2000.0f && !edgeActive && active <= 6;
      return output;
    }

    if (lostSinceMs_ == 0) lostSinceMs_ = nowMs;
    const uint32_t lostFor = nowMs - lostSinceMs_;
    if ((gapActive_ || gapArmed_) && lostFor <= gapBridgeMs) {
      gapActive_ = true;
      output.left = lastLeft_;
      output.right = lastRight_;
      output.valid = true;
      return output;
    }

    gapActive_ = false;
    gapArmed_ = false;
    // Search in the direction where an edge sensor saw the line most recently.
    if (lastEnd_ == 1) {
      output.left = lostFor < 300 ? sharpLeft : bendLeft;
      output.right = lostFor < 300 ? sharpRight : bendRight;
    } else {
      output.left = lostFor < 300 ? sharpRight : bendRight;
      output.right = lostFor < 300 ? sharpLeft : bendLeft;
    }
    output.valid = true;
    return output;
  }

 private:
  bool running_ = false;
  bool gapArmed_ = false;
  bool gapActive_ = false;
  uint8_t activeCount_ = 0;
  uint8_t lastEnd_ = 0;
  float position_ = 8500.0f;
  float error_ = 0.0f;
  float lastError_ = 0.0f;
  float lastLeft_ = 0.0f;
  float lastRight_ = 0.0f;
  uint32_t lostSinceMs_ = 0;
};
