#pragma once

#include <cstdint>

#include "motor.hpp"

namespace drivers
{

/**
 * @class DRV8833
 * @brief Hardware backend for the DRV8833 dual H-bridge motor driver. The chip has two H-bridges
 * (A and B), and each Side gets its own instance driving one of them: left = bridge A, right =
 * bridge B. Each bridge is controlled by PWM-ing its two inputs (xIN1, xIN2) from the ESP's LEDC
 * peripheral.
 */
class DRV8833 : public Motor
{
public:
  DRV8833(Side side);

  /// Sets up the shared PWM timer and this bridge's two PWM channels. The motor coasts until the
  /// first publish().
  void start() override;
  /// Drives the wheel at rad_s (open loop). Positive is forward, 0 brakes the motor.
  /// TODO: The velocity PID (PWMController, #616) outputs a [-1, 1] duty. If it goes in front of the motor,
  /// this should take that duty directly instead of rad/s. Needs a team decision since it also changes
  /// motor.hpp, Motor_SITL and Payload.
  void publish(double rad_s) override;

private:
  /// Writes raw LEDC duty counts to this bridge's xIN1 / xIN2 pins.
  void write_inputs(uint32_t in1_duty, uint32_t in2_duty);

  const Side side_;
  bool started_ = false;
};

}  // namespace drivers
