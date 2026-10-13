#include "motor_hw.hpp"

namespace drivers
{

Motor_HW::Motor_HW(PayloadCorner corner) : Motor(corner) {}

void Motor_HW::start()
{
  // TODO
}

void Motor_HW::publish(double)
{
  // TODO
}

std::unique_ptr<Motor> Motor::create(PayloadCorner corner)
{
  return std::make_unique<Motor_HW>(corner);
}

}  // namespace drivers
