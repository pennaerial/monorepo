#pragma once

#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stdexcept>
#include <string>

#include "image_transport/image_transport.hpp"
#include "pennair_vision/ros_message.hpp"
#include "pennair_vision/vision_plugin.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using CameraImage = sensor_msgs::msg::Image::ConstSharedPtr;
using CameraInfo = sensor_msgs::msg::CameraInfo::ConstSharedPtr;

template <pennair_vision::RosMessage MessageT>
struct ProcessResult {
  MessageT message;
  /// Optional debug image; leave empty when debug output is not requested.
  cv::Mat debug_image;
  /// ROS image encoding matching the debug image pixels.
  std::string debug_encoding{sensor_msgs::image_encodings::BGR8};
};

namespace pennair_vision
{

template <RosMessage MessageT>
class LazyCameraPublisher : public VisionPlugin
{
public:
  LazyCameraPublisher() = default;
  virtual ~LazyCameraPublisher() = default;

  /// Check VisionPlugin
  void initialize(rclcpp::Node& node, const std::string& camera_topic, double debug_image_rate) override;

  /// Subscribes to camera frames and processes them when output subscribers exist. See VisionPlugin
  /// Processes at full speed for result subscribers, or at the debug rate for debug-only subscribers
  void start() override;

  /// Runs the CV alg implementation and returns the ProcessResult type
  virtual ProcessResult<MessageT> process(const CameraImage& image, const CameraInfo& info, bool generate_debug) = 0;

protected:
  /// Camera topic to subscribe to
  std::string camera_topic_;
  // INVARIANT: vision manager node will be valid for the lifetime of the plugin
  rclcpp::Node* node_{nullptr};

private:
  /// Wall-clock limit, independent of camera timestamps or simulation time jumps.
  std::chrono::duration<double> debug_period_{0.0};
  std::optional<std::chrono::steady_clock::time_point> last_debug_time_;
  /// Publishes result of cv object
  typename rclcpp::Publisher<MessageT>::SharedPtr cv_result_pub_;
  /// Object for lazily publishing debug camera images
  image_transport::Publisher cv_debug_pub_;
  /// Object for subscribing to raw camera frames
  image_transport::CameraSubscriber camera_sub_;
};

template <RosMessage MessageT>
void LazyCameraPublisher<MessageT>::initialize(
    rclcpp::Node& node,
    const std::string& camera_topic,
    double debug_image_rate
)
{
  /// convert hz to seconds
  debug_period_ = std::chrono::duration<double>(debug_image_rate > 0.0 ? 1.0 / debug_image_rate : 0.0);
  last_debug_time_.reset();
  camera_topic_ = camera_topic;
  node_ = &node;
  cv_debug_pub_ = image_transport::create_publisher(node_, "/cv/debug", rclcpp::QoS(1).get_rmw_qos_profile());
  cv_result_pub_ = node_->create_publisher<MessageT>("/cv/result", 10);

  start();
}

/// Subscribes to camera topic; gate expensive processing and debug generation before calling the plugin
template <RosMessage MessageT>
void LazyCameraPublisher<MessageT>::start()
{
  camera_sub_ = image_transport::create_camera_subscription(
      node_, camera_topic_,
      [this](const CameraImage& image, const CameraInfo& info) {
        const bool want_result = cv_result_pub_->get_subscription_count() > 0;
        const bool want_debug = cv_debug_pub_.getNumSubscribers() > 0;
        const auto now = std::chrono::steady_clock::now();
        const bool generate_debug = want_debug && (!last_debug_time_ || now - *last_debug_time_ >= debug_period_);
        if (!want_result && !generate_debug) {
          // don't process image if no subscribers
          return;
        }

        if (generate_debug) {
          // Rate-limit attempts too, even if the plugin returns no debug image.
          last_debug_time_ = now;
        }
        ProcessResult<MessageT> result = process(image, info, generate_debug);
        if (generate_debug && !result.debug_image.empty()) {
          cv_debug_pub_.publish(
              cv_bridge::CvImage(image->header, result.debug_encoding, result.debug_image).toImageMsg()
          );
        }
        if (want_result) {
          cv_result_pub_->publish(result.message);
        }
      },
      "raw", rclcpp::QoS(1).get_rmw_qos_profile()
  );
}

}  // namespace pennair_vision
