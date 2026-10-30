#pragma once

#include "motor.hpp"

namespace drivers
{

class Motor_HW : public Motor
{
public:
  Motor_HW(Side side);

  void start() override;
  void publish(double pwm) override;
};

}  // namespace drivers
