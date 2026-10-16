#include <chrono>
#include <image_transport/camera_publisher.hpp>
#include <memory>

#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

// namespace pennair_vision_constants
// {
// constexpr uint32_t QUEUE_SIZE = 1;
// }

namespace pennair_vision
{
class LazyCameraPublisher
{
public:
  LazyCameraPublisher(const rclcpp::Node::SharedPtr& node) : vision_manager_(node)
  {
    it_ = std::make_shared<image_transport::ImageTransport>(vision_manager_->shared_from_this());
    camera_pub_ = it_->advertiseCamera("camera/image_raw", 1);

    callback_timer_ = vision_manager_->create_wall_timer(std::chrono::milliseconds(33), [this]() { timer_callback(); });
  }
  virtual ~LazyCameraPublisher() = default;

private:
  /// The method that'll be called when the timer expires
  void timer_callback();

  // I think it's more idomatic for them to just be const rclcpp::Node instead of SharedPtr, but then that'd
  // mean * for every single method call; not too sure what to do about that

  /// A method for getting the camera hardware's frame
  sensor_msgs::msg::Image capture_hardware_frame();

  /// Grabs the camera's calibration information
  sensor_msgs::msg::CameraInfo get_camera_calibration();

  // INVARIANT: vision manager will be valid for the lifetime of the plugin
  const rclcpp::Node::SharedPtr& vision_manager_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher camera_pub_;
  rclcpp::TimerBase::SharedPtr callback_timer_;
};

}  // namespace pennair_vision
