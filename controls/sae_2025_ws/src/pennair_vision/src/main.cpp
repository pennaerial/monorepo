#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "pennair_vision/vision_manager.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("vision_manager");
  pennair_vision::VisionManager vision_manager(node);
  vision_manager.init_plugins();

  // multithreaded executor lets callbacks run in parallel
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
