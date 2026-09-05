#include "vision_manager/vision_manager.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace pennair_vision
{

VisionManager::VisionManager() : Node("vision_manager"), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  auto timer_callback = [this]() -> void {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(this->count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    this->publisher_->publish(message);
  };
  timer_ = this->create_wall_timer(500ms, timer_callback);
}

}  // namespace pennair_vision
