#include "pennair_vision/vision_manager.hpp"

#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace pennair_vision
{

VisionManager::VisionManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("vision_manager", options), plugin_loader_("pennair_vision", "pennair_vision::VisionPlugin")
{
  param_listener_ = std::make_shared<vision_manager::ParamListener>(this);
  params_ = param_listener_->get_params();  // initialize params
}

void VisionManager::init_plugins()
{
  // initialize a single plugin by name (e.g. pennair_vision::BasicVision) and add it to the plugin_map_
  auto initialize_plugin = [this](const std::string& plugin_name) {
    try {
      RCLCPP_DEBUG(this->get_logger(), "Initializing plugin: %s", plugin_name.c_str());
      plugin_map_[plugin_name].plugin_instance_ = plugin_loader_.createUniqueInstance(plugin_name);
      plugin_map_[plugin_name].plugin_instance_->initialize(*this, params_.camera_topic);
      // TODO: add better error handling for plugins that don't exist or fail to initialize
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(get_logger(), "Failed to create plugin: %s", ex.what());
    }
  };

  if (params_.plugins.empty()) {
    RCLCPP_WARN(this->get_logger(), "No plugins specified at startup!");
  }

  for (const std::string& plugin_name : params_.plugins) {
    initialize_plugin(plugin_name);
  }
}

}  // namespace pennair_vision

namespace rclcpp_components
{

template <>
class NodeFactoryTemplate<pennair_vision::VisionManager> : public NodeFactory
{
public:
  NodeInstanceWrapper create_node_instance(const rclcpp::NodeOptions& options) override
  {
    auto node = std::make_shared<pennair_vision::VisionManager>(options);
    node->init_plugins();

    return NodeInstanceWrapper(node, [](const std::shared_ptr<void>& instance) {
      return std::static_pointer_cast<pennair_vision::VisionManager>(instance)->get_node_base_interface();
    });
  }
};

}  // namespace rclcpp_components

RCLCPP_COMPONENTS_REGISTER_NODE(pennair_vision::VisionManager)
