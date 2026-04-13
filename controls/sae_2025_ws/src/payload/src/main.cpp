#include "rclcpp/rclcpp.hpp"
#include "payload/payload.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Payload>("vehicle_main");
  node->init();
  rclcpp::spin(node);
  node.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
