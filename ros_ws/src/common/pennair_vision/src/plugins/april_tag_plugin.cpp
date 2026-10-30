#include "pennair_vision/plugins/april_tag_plugin.hpp"

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "pennair_vision/vision_plugin.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace cv;

namespace pennair_vision
{

AprilTagPlugin::AprilTagPlugin()
{
  tf_ = tag36h11_create();
  td_ = apriltag_detector_create();
  apriltag_detector_add_family(td_, tf_);

  td_->quad_decimate = 2.0;
  td_->quad_sigma = 0.0;
  td_->nthreads = 2;
  td_->refine_edges = 1;
}

AprilTagPlugin::~AprilTagPlugin()
{
  if (td_)
    apriltag_detector_destroy(td_);
  if (tf_)
    tag36h11_destroy(tf_);
}

ProcessResult<std_msgs::msg::String>
AprilTagPlugin::process(const CameraImage& image, const CameraInfo& /* info */, bool generate_debug)
{
  ProcessResult<std_msgs::msg::String> result;
  count_++;

  const auto frame = cv_bridge::toCvShare(image, "bgr8");
  cv::Mat gray;
  cv::cvtColor(frame->image, gray, cv::COLOR_BGR2GRAY);

  image_u8_t apriltag_img = {.width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data};

  zarray_t* detections = apriltag_detector_detect(td_, &apriltag_img);
  int num_tags = zarray_size(detections);

  std::string detected_ids = "";
  for (int i = 0; i < num_tags; i++) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);
    detected_ids += std::to_string(det->id) + (i == num_tags - 1 ? "" : ", ");
  }

  result.message.data = std::format("Frame #{}, Tags Number: [{}]", count_, detected_ids);

  if (generate_debug) {
    result.debug_image = gray.clone();
    result.debug_encoding = sensor_msgs::image_encodings::MONO8;
  }

  apriltag_detections_destroy(detections);

  return result;
}

};  // namespace pennair_vision

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pennair_vision::AprilTagPlugin, pennair_vision::VisionPlugin)