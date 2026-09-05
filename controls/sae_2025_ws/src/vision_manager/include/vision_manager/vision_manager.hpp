#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include "std_msgs/msg/string.hpp"

namespace pennair_vision
{

class VisionManager : public rclcpp::Node
{
public:
  VisionManager();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

} // namespace pennair_vision
