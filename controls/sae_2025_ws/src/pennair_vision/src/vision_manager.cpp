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
  plugin_instance_ = plugin_loader_.createUniqueInstance("pennair_vision::BasicVision");
  plugin_instance_->initialize(node_);
}

}  // namespace pennair_vision
