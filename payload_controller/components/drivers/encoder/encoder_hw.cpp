#include "encoder_hw.hpp"

namespace drivers {

void Encoder_HW::start()
{

}
void Encoder_HW::publish_motor_right(double)
{

}
void Encoder_HW::publish_motor_left(double)
{

}

Encoder* Encoder::instance()
{
  static Encoder_HW instance;
  return &instance;
}

} // namespace drivers
