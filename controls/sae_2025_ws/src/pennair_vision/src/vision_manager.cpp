#include "pennair_vision/vision_manager.hpp"

#include <string>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace pennair_vision
{

VisionManager::VisionManager(rclcpp::Node::SharedPtr node)
    : node_(node), plugin_loader_("pennair_vision", "pennair_vision::VisionPlugin")
{
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
      plugin_map_[plugin_name] = plugin_loader_.createUniqueInstance(plugin_name);
      plugin_map_[plugin_name]->initialize(node_);
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to create plugin: %s", ex.what());
    }
  };

  for (const std::string& plugin_name : plugin_names) {
    initialize_plugin(plugin_name);
  }
}

}  // namespace pennair_vision
