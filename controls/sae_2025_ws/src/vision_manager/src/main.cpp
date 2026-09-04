#include <rclcpp/rclcpp.hpp>

#include "vision_manager/vision_manager.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pennair_vision::VisionManager>());
  rclcpp::shutdown();
  return 0;
}
