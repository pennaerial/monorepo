#include "encoder_hw.hpp"

namespace drivers
{

Encoder_HW::Encoder_HW(Side)
{
  // TODO
}


void Encoder_HW::start()
{
  // TODO
}

void Encoder_HW::update()
{
  // TODO Read this side's hardware encoder and cache ticks plus output-shaft
  // angular velocity in rad/s using write_latest(ticks, angular_velocity_rad_s).
}

Encoder& Encoder::instance(Side side)
{
  static Encoder_HW left(Side::LEFT);
  static Encoder_HW right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}

}  // namespace drivers
