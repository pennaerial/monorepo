#pragma once

namespace drivers
{

struct MotorConfig {
  /// Gazebo command topic under /model/<model>/ that this motor publishes to (SITL only).
  const char* sitl_command_topic;
};

class Motor
{
public:
  virtual ~Motor() = default;

  virtual void start() = 0;
  virtual void publish(double rad_s) = 0;

protected:
  Motor() = default;
};

}  // namespace drivers
