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

class VisionManager : public rclcpp::Node
{
public:
  explicit VisionManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// Creates plugins specified at startup from ROS params
  void init_plugins();

  // TODO: create a method to return the list of active plugins
private:
  /// ClassLoader for dynamically creating VisionPlugins
  pluginlib::ClassLoader<VisionPlugin> plugin_loader_;
  /// Defers plugin initialization until the component is owned by a shared_ptr.
  rclcpp::TimerBase::SharedPtr initialization_timer_;
  /// Periodic status log for the manager component.
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  /// VisionPlugin instance; key is the plugin name, value is the plugin instance
  VisionPluginMap plugin_map_;
};

}  // namespace pennair_vision
