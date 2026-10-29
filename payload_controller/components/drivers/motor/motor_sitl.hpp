#pragma once

#include <gz/msgs/double.pb.h>
#include <gz/msgs/model.pb.h>

#include <atomic>
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
  // Publish a PWM command
  void publish(double pwm) override;

private:
  void make_motor_advertiser(char* buf, std::size_t size);
  // Reads the motor joint state (for getting angular velocity directly to use in torque calculations from PWM)
  void on_joint_state_msg(const gz::msgs::Model& msg);

  // Generates a motor torque for gazebo from an input PWM command
  float generate_motor_torque(float pwm);

  const sitl::SimConfig sitl_config_;
  /// Command topic under /model/<model>/ that this motor publishes to.
  const char* command_topic_;
  const char* joint_name_;
  /// Gazebo reports output-shaft speed in rad/s; zero until the first sample.
  std::atomic<double> wheel_angular_velocity_rad_s_{0.0};
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher motor_publisher_;
};

}  // namespace drivers
