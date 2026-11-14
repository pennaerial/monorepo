#include "pennair_vision/plugins/vision_republisher.hpp"

#include "pennair_vision/vision_plugin.hpp"

namespace pennair_vision
{

ProcessResult<std_msgs::msg::Empty>
VisionRepublisher::process(const CameraImage& image, const CameraInfo&, bool generate_debug)
{
  ProcessResult<std_msgs::msg::Empty> result;
  if (!generate_debug) {
    return result;
  }
  try {
    result.debug_image = cv_bridge::toCvShare(image)->image;
    result.debug_encoding = image->encoding;
  } catch (const cv_bridge::Exception& ex) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "VisionRepublisher: cannot wrap %s image: %s",
        image->encoding.c_str(), ex.what()
    );
  }
  return result;
}

};  // namespace pennair_vision

#include <pluginlib/class_list_macros.hpp>
// export as a VisionPlugin
PLUGINLIB_EXPORT_CLASS(pennair_vision::VisionRepublisher, pennair_vision::VisionPlugin)
