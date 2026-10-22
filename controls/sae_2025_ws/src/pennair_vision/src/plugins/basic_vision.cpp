#include "pennair_vision/plugins/basic_vision.hpp"

#include "pennair_vision/vision_plugin.hpp"
#include "rclcpp/logger.hpp"


namespace pennair_vision
{

ProcessResult<std_msgs::msg::Int32> BasicVision::process(
    const CameraImage image,
    const CameraInfo /* info */
)
{
  return {std_msgs::msg::Int32{}, nullptr};
}

};  // namespace pennair_vision

#include <pluginlib/class_list_macros.hpp>
// export as a VisionPlugin
PLUGINLIB_EXPORT_CLASS(pennair_vision::BasicVision, pennair_vision::VisionPlugin)
