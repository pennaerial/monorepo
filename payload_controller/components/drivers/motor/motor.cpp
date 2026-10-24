#include "motor.hpp"

#include "sdkconfig.h"

#if defined(CONFIG_IDF_TARGET_LINUX)
#include "motor_sitl.hpp"
using MotorBackend = drivers::Motor_SITL;
#else
#include "motor_hw.hpp"
using MotorBackend = drivers::Motor_HW;
#endif

namespace drivers
{

Motor& get_motor(Side side)
{
  static MotorBackend left(Side::LEFT);
  static MotorBackend right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}

}  // namespace drivers
