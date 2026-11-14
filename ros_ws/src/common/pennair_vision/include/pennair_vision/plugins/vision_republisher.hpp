#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pennair_vision/lazy_camera_publisher.hpp"
#include "std_msgs/msg/empty.hpp"

namespace pennair_vision
{

/// Republishes the original camera frames on /cv/debug, throttled to debug_image_rate.
/// Frames go out through the debug path rather than as a result, so a result subscriber does not
/// trigger full-rate republishing. The result is Empty because this plugin has no algorithm output.
class VisionRepublisher : public LazyCameraPublisher<std_msgs::msg::Empty>
{
public:
  /// See LazyCameraPublisher
  ProcessResult<std_msgs::msg::Empty> process(const CameraImage& image, const CameraInfo& info, bool generate_debug)
      override;
};

};  // namespace pennair_vision
