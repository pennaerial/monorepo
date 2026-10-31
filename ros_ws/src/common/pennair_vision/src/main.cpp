#include <rclcpp/rclcpp.hpp>

#include "pennair_vision/vision_manager.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // make the video manager shared ptr
  auto vision_manager = std::make_shared<pennair_vision::VisionManager>();
  vision_manager->init_plugins();

  // multithreaded executor lets callbacks run in parallel
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(vision_manager);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
