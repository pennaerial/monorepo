#pragma once

#include "encoder.hpp"

namespace drivers
{

class Encoder_HW : public Encoder
{
public:
  explicit Encoder_HW(Side side);
  /// See encoder.hpp
  void start() override;
};

}  // namespace drivers
