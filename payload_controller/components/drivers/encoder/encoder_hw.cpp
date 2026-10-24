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

Encoder& get_encoder(Side side)
{
  static Encoder_HW left(Side::LEFT);
  static Encoder_HW right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}

}  // namespace drivers
