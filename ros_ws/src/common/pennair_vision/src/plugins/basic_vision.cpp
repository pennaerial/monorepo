#include "pennair_vision/plugins/basic_vision.hpp"

#include "pennair_vision/vision_plugin.hpp"


namespace pennair_vision
{

ProcessResult<std_msgs::msg::Int32>
BasicVision::process(const CameraImage& image, const CameraInfo& /* info */, bool generate_debug)
{
  ProcessResult<std_msgs::msg::Int32> result;
  result.message.data = ++count_;
  if (generate_debug) {
    RCLCPP_DEBUG(node_->get_logger(), "BasicVision: generating debug image!");
    const auto frame = cv_bridge::toCvShare(image, "bgr8");
    cv::Mat gray;
    cv::cvtColor(frame->image, result.debug_image, cv::COLOR_BGR2GRAY);
    result.debug_encoding = sensor_msgs::image_encodings::MONO8;
  }
  return result;
}

};  // namespace pennair_vision

#include <pluginlib/class_list_macros.hpp>
// export as a VisionPlugin
PLUGINLIB_EXPORT_CLASS(pennair_vision::BasicVision, pennair_vision::VisionPlugin)
