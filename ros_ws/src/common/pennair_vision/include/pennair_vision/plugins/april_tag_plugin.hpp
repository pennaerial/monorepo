#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "pennair_vision/lazy_camera_publisher.hpp"
#include "std_msgs/msg/string.hpp"

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

namespace pennair_vision
{

class AprilTagPlugin : public LazyCameraPublisher<std_msgs::msg::String>
{
public:
  AprilTagPlugin();
  ~AprilTagPlugin() override;
  ProcessResult<std_msgs::msg::String> process(const CameraImage& image, const CameraInfo& info, bool generate_debug)
      override;

private:
  ::apriltag_detector_t *td_{nullptr};
  ::apriltag_family_t *tf_{nullptr};  
  int count_ = 0;
};

};  // namespace pennair_vision
