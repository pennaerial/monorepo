#include "payload/payload.hpp"

Payload::Payload(const std::string& payload_name)
: rclcpp::Node(payload_name) {
    payload_name_ = payload_name;

    payload_params_listener_ = std::make_shared<payload::ParamListener>(this);
    payload_params_ = payload_params_listener_->get_params();
    RCLCPP_INFO(this->get_logger(), payload_params_.controller.c_str());
    // this->get_logger
}

void Payload::drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg) {
    double linear_v = msg->linear;
    double angular_v = msg->angular;
    controller_->drive_command(linear_v, angular_v);
}



