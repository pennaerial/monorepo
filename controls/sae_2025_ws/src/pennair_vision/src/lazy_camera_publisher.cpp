#include "pennair_vision/lazy_camera_publisher.hpp"

using namespace pennair_vision;

void LazyCameraPublisher::timer_callback()
{
  // return early
  if (camera_pub_.getNumSubscribers() == 0) {
    return;
  }

  sensor_msgs::msg::Image img_msg = capture_hardware_frame();
  sensor_msgs::msg::CameraInfo info_msg = get_camera_calibration();

  camera_pub_.publish(img_msg, info_msg);
}

sensor_msgs::msg::Image LazyCameraPublisher::capture_hardware_frame()
{
  // NOTE: this is not complete, still need to use cv2 to get actual camera data
  sensor_msgs::msg::Image img;
  img.header.stamp = vision_manager_->now();
  img.header.frame_id = "camera_optical_frame";
  return img;
}

sensor_msgs::msg::CameraInfo LazyCameraPublisher::get_camera_calibration()
{
  // NOTE: this is not complete, still need to use cv2 to get actual camera data
  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = vision_manager_->now();
  info.header.frame_id = "camera_optical_frame";
  return info;
}
