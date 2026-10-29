#pragma once
#include "payload_corner.hpp"

namespace drivers
{

class Motor
{
public:
  virtual ~Motor() = default;

  virtual void start() = 0;
  /// Commands a pwm command to the motors
  virtual void publish(double pwm) = 0;
  /// Gets the motor instance for the given side (sitl, hardware). implemented in backend .cpp
  /// files, not motor.cpp
  static Motor& instance(Side side);

protected:
  Motor() = default;
};

}  // namespace drivers
