#include "pennair_vision/vision_manager.hpp"

#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace pennair_vision
{

VisionManager::VisionManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("vision_manager", options),
      plugin_loader_("pennair_vision", "pennair_vision::VisionPlugin")
{
  initialization_timer_ = create_wall_timer(0ms, [this]() {
    initialization_timer_->cancel();
    init_plugins();
  });

  // TEMP: continuous logging
  heartbeat_timer_ = create_wall_timer(0.67s, [this]() {
    RCLCPP_INFO(get_logger(), "VisionManager is running");
  });
}

void VisionManager::init_plugins()
{
  // TODO: read this list dynamically from ROS params
  std::vector<std::string> plugin_names = {"pennair_vision::BasicVision"};

  // this lambda function will be used to initialize each plugin in the list and
  // add it to the plugin_map_

  // TODO: add better error handling for plugins that don't exist or fail to initialize
  auto initialize_plugin = [this](const std::string& plugin_name) {
    try {
      plugin_map_[plugin_name].plugin_instance_ = plugin_loader_.createUniqueInstance(plugin_name);
      plugin_map_[plugin_name].plugin_instance_->initialize(shared_from_this());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(get_logger(), "Failed to create plugin: %s", ex.what());
    }
  };

  for (const std::string& plugin_name : plugin_names) {
    initialize_plugin(plugin_name);
  }
}

}  // namespace pennair_vision

RCLCPP_COMPONENTS_REGISTER_NODE(pennair_vision::VisionManager)
