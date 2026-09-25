#include "encoder_hw.hpp"

namespace drivers
{

void Encoder_HW::start()
{
  // TODO
}
void Encoder_HW::publish_motor_right(double)
{
  // TODO
}
void Encoder_HW::publish_motor_left(double)
{
  // TODO
}

Encoder* Encoder::instance()
{
  static Encoder_HW instance;
  return &instance;
}

}  // namespace drivers
