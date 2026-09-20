#pragma once

#include <map>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pennair_vision/vision_plugin.hpp"

namespace pennair_vision
{

class VisionManager
{
public:
  VisionManager(rclcpp::Node::SharedPtr node);

  /// Creates plugins specified at startup from ROS params
  void init_plugins();

  // TODO: create a method to return the list of active plugins
private:
  /// ROS node ptr
  rclcpp::Node::SharedPtr node_;
  /// ClassLoader for dynamically creating VisionPlugins
  pluginlib::ClassLoader<VisionPlugin> plugin_loader_;
  /// VisionPlugin instance; key is the plugin name, value is the plugin instance
  std::map<std::string, pluginlib::UniquePtr<VisionPlugin>> plugin_map_;
};

}  // namespace pennair_vision
