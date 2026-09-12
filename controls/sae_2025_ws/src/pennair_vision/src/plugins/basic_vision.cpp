#include "pennair_vision/plugins/basic_vision.hpp"

#include "pennair_vision/vision_plugin.hpp"
#include "rclcpp/logger.hpp"


namespace pennair_vision
{

void BasicVision::initialize(const rclcpp::Node::SharedPtr& node)
{
  RCLCPP_INFO(node->get_logger(), "BasicVision plugin started!");
}

void BasicVision::process(sensor_msgs::msg::Image::ConstSharedPtr image) {}


};  // namespace pennair_vision

#include <pluginlib/class_list_macros.hpp>
// export as a VisionPlugin
PLUGINLIB_EXPORT_CLASS(pennair_vision::BasicVision, pennair_vision::VisionPlugin)
