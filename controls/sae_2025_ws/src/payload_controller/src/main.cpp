#include "rclcpp/rclcpp.hpp"
#include "payload_controller/payload_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Payload>("vehicle_main");
  rclcpp::on_shutdown(
    [weak_node = std::weak_ptr<Payload>(node)]() {
      if (auto locked_node = weak_node.lock()) {
        locked_node->prepare_for_shutdown();
      }
    });
  node->init();
  rclcpp::spin(node);
  node->prepare_for_shutdown();
  node.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
