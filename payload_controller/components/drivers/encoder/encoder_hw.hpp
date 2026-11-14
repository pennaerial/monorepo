#pragma once

#include "encoder.hpp"

namespace drivers
{

class Encoder_HW : public Encoder
{
public:
  Encoder_HW(Side side);
  /// See encoder.hpp
  void start() override;
  /// Called by this side's encoder task, matching the simulation backend.
  void update() override;
};

}  // namespace drivers
