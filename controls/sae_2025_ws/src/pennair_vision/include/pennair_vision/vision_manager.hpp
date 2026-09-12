#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include "std_msgs/msg/string.hpp"
#include "pennair_vision/vision_plugin.hpp"

namespace pennair_vision
{

class VisionManager
{
public:
  VisionManager(rclcpp::Node::SharedPtr node);

  /** Creates plugins specified at startup from ROS params */
  void init_plugins();

private:
  /// ROS node ptr
  rclcpp::Node::SharedPtr node_;
  /// ClassLoader for dynamically creating VisionPlugins
  pluginlib::ClassLoader<VisionPlugin> plugin_loader_;

  pluginlib::UniquePtr<VisionPlugin> plugin_instance_;

};

}  // namespace pennair_vision
