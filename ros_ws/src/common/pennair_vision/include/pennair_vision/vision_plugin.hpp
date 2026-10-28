#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace pennair_vision
{

class VisionPlugin
{
public:
  virtual ~VisionPlugin() = default;

  /// Initialize the plugin by passing in ROS node
  virtual void initialize(rclcpp::Node& node, const std::string& topic) = 0;

  virtual void start() = 0;
};

}  // namespace pennair_vision
