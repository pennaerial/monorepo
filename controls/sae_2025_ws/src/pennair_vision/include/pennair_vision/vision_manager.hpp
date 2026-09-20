#pragma once

#include <map>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pennair_vision/vision_plugin.hpp"

struct VisionPluginStruct {
  // add more relevant fields when necessary
  pluginlib::UniquePtr<pennair_vision::VisionPlugin> plugin_instance_;
};

using VisionPluginMap = std::map<std::string, VisionPluginStruct>;

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
  VisionPluginMap plugin_map_;
};

}  // namespace pennair_vision
