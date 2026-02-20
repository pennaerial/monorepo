#include "payload/payload.hpp"

Payload::Payload(const std::string& payload_name)
: rclcpp::Node(payload_name) {
    payload_name_ = this->get_name(); //allows for override from launch file node name

    payload_params_listener_ = std::make_shared<payload::ParamListener>(this);
    payload_params_ = payload_params_listener_->get_params();

    std::string ros_drive_topic = "/" + payload_name_ + "/cmd_drive";
    ros_drive_subscriber_ = this->create_subscription<payload_interfaces::msg::DriveCommand>(
        ros_drive_topic, 10, std::bind(&Payload::drive_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", ros_drive_topic.c_str());


    if (payload_params_.controller == "SimController") { //move to enum
        controller_ = std::make_shared<SimController>();
    }
}

void Payload::init() {
    controller_->initialize(shared_from_this());
}

void Payload::drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg) {
    double linear_v = msg->linear;
    double angular_v = msg->angular;
    controller_->drive_command(linear_v, angular_v);
}



