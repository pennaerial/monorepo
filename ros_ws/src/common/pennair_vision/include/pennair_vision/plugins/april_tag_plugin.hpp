#pragma once

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "pennair_vision/lazy_camera_publisher.hpp"
#include "pennair_vision/msg/april_tag_array.hpp"

namespace pennair_vision
{

class AprilTagPlugin : public LazyCameraPublisher<pennair_vision::msg::AprilTagArray>
{
public:
  AprilTagPlugin();
  ~AprilTagPlugin() override;
  ProcessResult<pennair_vision::msg::AprilTagArray> process(const CameraImage& image, const CameraInfo& info, bool generate_debug)
      override;

private:
  apriltag_detector_t* td_{nullptr};
  apriltag_family_t* tf_{nullptr};
};

};  // namespace pennair_vision
