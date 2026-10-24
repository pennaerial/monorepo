#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pennair_vision/lazy_camera_publisher.hpp"
#include "std_msgs/msg/int32.hpp"

namespace pennair_vision
{

class BasicVision : public LazyCameraPublisher<std_msgs::msg::Int32>
{
public:
  /// See LazyCameraPublisher
  ProcessResult<std_msgs::msg::Int32> process(const CameraImage& image, const CameraInfo& info, bool generate_debug)
      override;
};

};  // namespace pennair_vision
