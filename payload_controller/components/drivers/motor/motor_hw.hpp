#pragma once

#include "motor.hpp"

namespace drivers
{

class Motor_HW : public Motor
{
public:
  explicit Motor_HW(PayloadCorner corner);

  void start() override;
  void publish(double rad_s) override;
};

}  // namespace drivers
