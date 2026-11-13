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

  /// Initialize plugin with a ROS node, camera topic and maximum debug rate in Hz (0 means unlimited).
  virtual void initialize(rclcpp::Node& node, const std::string& topic, double debug_rate) = 0;

  virtual void start() = 0;
};

}  // namespace pennair_vision
