#include "encoder_hw.hpp"

namespace drivers
{

Encoder_HW::Encoder_HW(PayloadCorner corner) : Encoder(corner) {}

void Encoder_HW::start()
{
  // TODO
}

std::unique_ptr<Encoder> Encoder::create(PayloadCorner corner)
{
  return std::make_unique<Encoder_HW>(corner);
}

}  // namespace drivers
