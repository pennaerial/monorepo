#pragma once

#include <cstdint>

#include "motor.hpp"

namespace drivers
{

/**
 * @class DRV8833
 * @brief Hardware backend for the DRV8833 dual H-bridge motor driver. The chip has two H-bridges
 * (A and B), and each Side gets its own instance driving one of them: left = bridge B, right =
 * bridge A. Each bridge is controlled by PWM-ing its two inputs (xIN1, xIN2) from the ESP's LEDC
 * peripheral.
 */
class DRV8833 : public Motor
{
public:
  DRV8833(Side side);

  /// Sets up the shared PWM timer and this bridge's two PWM channels. The motor coasts until the
  /// first publish().
  void start() override;
  /// Drives the wheel at a signed PWM duty in [-1, 1]. Positive is forward, 0 brakes the motor.
  void publish(double pwm) override;

private:
  /// Writes raw LEDC duty counts to this bridge's xIN1 / xIN2 pins.
  void write_inputs(uint32_t in1_duty, uint32_t in2_duty);

  const Side side_;
  bool started_ = false;
};

}  // namespace drivers
