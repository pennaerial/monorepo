#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pennair_vision/lazy_camera_publisher.hpp>

namespace pennair_vision
{

class VisionPlugin
{
public:
  virtual ~VisionPlugin() = default;

  /// Initialize the plugin by passing in ROS node
  virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::string& topic) = 0;

  virtual void start() = 0;
};

}  // namespace pennair_vision
