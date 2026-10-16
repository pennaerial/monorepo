#pragma once

#include <gz/msgs/actuators.pb.h>

#include <cstddef>
#include <gz/transport/Node.hh>

#include "motor.hpp"
#include "sitl_runtime.hpp"

namespace drivers
{

class Motor_SITL : public Motor
{
public:
  explicit Motor_SITL(PayloadCorner corner);

  void start() override;
  void publish(double rad_s) override;

private:
  void make_motor_advertiser(char* buf, std::size_t size);

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher motor_publisher_;
  const sitl::SimConfig sitl_config_;
};

}  // namespace drivers
