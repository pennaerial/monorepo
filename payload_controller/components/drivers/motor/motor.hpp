#pragma once

#include <memory>

#include "payload_corner.hpp"

namespace drivers
{

class Motor
{
public:
  virtual ~Motor() = default;

  virtual void start() = 0;
  virtual void publish(double rad_s) = 0;

  static std::unique_ptr<Motor> create(PayloadCorner corner);

  PayloadCorner corner() const;

protected:
  explicit Motor(PayloadCorner corner);

private:
  PayloadCorner corner_;
};

}  // namespace drivers
