#pragma once
#include "payload_corner.hpp"

namespace drivers
{

class Motor
{
public:
  virtual ~Motor() = default;

  virtual void start() = 0;
  /// Commands wheel-joint torque in N*m, positive along the joint axis.
  virtual void publish(double torque_nm) = 0;
  /// Gets the motor instance for the given side (sitl, hardware). implemented in backend .cpp
  /// files, not motor.cpp
  static Motor& instance(Side side);

protected:
  Motor() = default;
};

}  // namespace drivers
