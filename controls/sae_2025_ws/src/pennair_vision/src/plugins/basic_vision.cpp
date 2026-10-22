#include "pennair_vision/plugins/basic_vision.hpp"

#include "pennair_vision/vision_plugin.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/image.h"
#include "sensor_msgs/msg/image.hpp"


namespace pennair_vision
{

ProcessResult<std_msgs::msg::Int32>
BasicVision::process(const CameraImage image, const CameraInfo /* info */, bool generate_debug)
{
  ProcessResult<std_msgs::msg::Int32> result;
  if (generate_debug) {
    result.debug_image = cv_bridge::toCvCopy(image, "bgr8")->image;
  }
  return result;
}

};  // namespace pennair_vision

#include <pluginlib/class_list_macros.hpp>
// export as a VisionPlugin
PLUGINLIB_EXPORT_CLASS(pennair_vision::BasicVision, pennair_vision::VisionPlugin)
