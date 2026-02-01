#include "payload/payload_controller.hpp"

PayloadController::PayloadController(const std::string& payload_name)
: rclcpp::Node(payload_name + "_controller") {
    payload_name_ = payload_name;
}

void PayloadController::drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg) {
    double linear_v = msg->linear;
    double angular_v = msg->angular;
    controller_backend_->drive_command(linear_v, angular_v);
}



