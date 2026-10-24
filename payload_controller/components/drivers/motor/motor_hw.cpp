#include "motor_hw.hpp"

namespace drivers
{

Motor_HW::Motor_HW(Side)
{
  // TODO
}

void Motor_HW::start()
{
  // TODO
}

void Motor_HW::publish(double)
{
  // TODO
}

Motor& Motor::instance(Side side)
{
  static Motor_HW left(Side::LEFT);
  static Motor_HW right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}

}  // namespace drivers
