#include "motor.hpp"

namespace drivers
{

Motor::Motor(PayloadCorner corner) : corner_(corner) {}

PayloadCorner Motor::corner() const
{
  return corner_;
}

}  // namespace drivers
