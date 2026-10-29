#pragma once

#include <gz/msgs/double.pb.h>

#include <cstddef>
#include <gz/transport/Node.hh>

#include "motor.hpp"
#include "sitl_runtime.hpp"

namespace drivers
{

class Motor_SITL : public Motor
{
public:
  Motor_SITL(Side side);

  void start() override;
  void publish(double torque_nm) override;

private:
  void make_motor_advertiser(char* buf, std::size_t size);

  // Generates a motor torque for gazebo from an input PWM command
  float generate_motor_torque(float pwm);

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher motor_publisher_;
  const sitl::SimConfig sitl_config_;
  /// Command topic under /model/<model>/ that this motor publishes to.
  const char* command_topic_;
};

}  // namespace drivers
