#pragma once

#include "encoder.hpp"

namespace drivers {

class Encoder_HW : public Encoder {
public:
  /// See encoder.hpp
  void start() override;
  /// See encoder.hpp
  void publish_motor_right(double) override;
  /// See encoder.hpp
  void publish_motor_left(double) override;
};

} // namespace drivers
