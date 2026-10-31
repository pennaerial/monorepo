#include "payload_controller/sim_controller.hpp"


SimController::SimController() {}

void SimController::initialize(rclcpp::Node* node)
{
  gz_node_ = std::make_shared<gz::transport::Node>();

  payload_params_listener_ = std::make_shared<payload::ParamListener>(node);
  payload_params_ = payload_params_listener_->get_params();

  std::string sim_entity_name = node->get_name();
  std::string gz_drive_topic = "/model/" + sim_entity_name + "/cmd_vel";

  gz_drive_publisher_ = gz_node_->Advertise<gz::msgs::Twist>(gz_drive_topic);
  RCLCPP_INFO(node->get_logger(), "SIM | Publishing gz drive commands to %s", gz_drive_topic.c_str());
}

void SimController::publish_drive_command(double linear, double angular)
{
  if (!gz_node_) {
    return;
  }

  gz::msgs::Twist msg;
  msg.mutable_linear()->set_x(linear);    // head-on direction
  msg.mutable_angular()->set_z(angular);  // positive for left, negative for right
  gz_drive_publisher_.Publish(msg);
}

void SimController::drive_command(double linear, double angular)
{
  if (shutdown_requested_.load()) {
    return;
  }
  publish_drive_command(linear, angular);
}

void SimController::servo_command(double degree)
{
  if (shutdown_requested_.load()) {
    return;
  }
  (void)degree;
}

void SimController::safe_shutdown()
{
  if (shutdown_requested_.exchange(true)) {
    return;
  }
  publish_drive_command(0.0, 0.0);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(SimController, Controller)
