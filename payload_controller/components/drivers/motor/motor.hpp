#pragma once
#include "payload_corner.hpp"

namespace drivers
{

class Motor
{
public:
  virtual ~Motor() = default;

  virtual void start() = 0;
  virtual void publish(double rad_s) = 0;

protected:
  Motor() = default;
};

/// Returns this build's motor for the given side. Defined by the backend CMake compiles.
Motor& get_motor(Side side);

}  // namespace drivers
