#pragma once

#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <string>

#include "image_transport/image_transport.hpp"
#include "pennair_vision/vision_plugin.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using CameraImage = sensor_msgs::msg::Image::ConstSharedPtr;
using CameraInfo = sensor_msgs::msg::CameraInfo::ConstSharedPtr;

template <typename MessageT>
struct ProcessResult {
  MessageT message;
  sensor_msgs::msg::Image::SharedPtr debug_image;
};

namespace pennair_vision
{

template <typename MessageT>
class LazyCameraPublisher : public VisionPlugin
{
public:
  LazyCameraPublisher() = default;
  virtual ~LazyCameraPublisher() = default;

  /// Check VisionPlugin
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& camera_topic) override;

  /// Subscribes to camera frames and processes them when output subscribers exist. See VisionPlugin
  void start() override;

  /// Runs the CV alg implementation and returns the ProcessResult type
  virtual ProcessResult<MessageT> process(const CameraImage image, const CameraInfo info) = 0;


private:
  /// Camera topic to subscribe to
  std::string camera_topic_;
  // INVARIANT: vision manager will be valid for the lifetime of the plugin
  rclcpp::Node::SharedPtr node_;
  /// Publishes result of cv object
  typename rclcpp::Publisher<MessageT>::SharedPtr cv_result_pub_;
  /// image transport object, creates CameraPublisher and CameraSubscriber
  std::shared_ptr<image_transport::ImageTransport> it_;
  /// Object for lazily publishing debug camera images
  image_transport::Publisher cv_debug_pub_;
  /// Object for subscribing to raw camera frames
  image_transport::CameraSubscriber camera_sub_;
  rclcpp::TimerBase::SharedPtr callback_timer_;
};

template <typename MessageT>
void LazyCameraPublisher<MessageT>::initialize(const rclcpp::Node::SharedPtr& node, const std::string& camera_topic)
{
  camera_topic_ = camera_topic;
  node_ = node;
  it_ = std::make_shared<image_transport::ImageTransport>(node_->shared_from_this());
  cv_debug_pub_ = it_->advertise("/cv/debug", 1);

  start();
}

/// Subscribes to camera frames and processes them when output subscribers exist
template <typename MessageT>
void LazyCameraPublisher<MessageT>::start()
{
  camera_sub_ = it_->subscribeCamera(camera_topic_, 1, [this](const CameraImage image, const CameraInfo info) {
    if (cv_result_pub_->get_subscription_count() > 0 || cv_debug_pub_.getNumSubscribers() > 0) {
      ProcessResult<MessageT> result = process(image, info);
      if (cv_debug_pub_.getNumSubscribers() > 0) {
        cv_debug_pub_.publish(result.debug_image);
      }
      cv_result_pub_->publish(result.message);
    }
  });
}

}  // namespace pennair_vision
