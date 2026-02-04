#include "payload/sim_controller.hpp"

SimController::SimController(const std::string& gz_drive_topic, rclcpp::Logger logger)
: logger_(logger) {
    logger_ = logger;
    gz_node_ = std::make_shared<gz::transport::Node>();
    gz_drive_publisher_ = gz_node_->Advertise<gz::msgs::Twist>(gz_drive_topic);
    RCLCPP_INFO(logger_, "Publishing gz drive commands to %s", gz_drive_topic.c_str());
}

void SimController::drive_command(double linear, double angular) {
    gz::msgs::Twist msg;
    msg.mutable_linear()->set_x(linear); //head-on direction
    msg.mutable_angular()->set_z(angular); //positive for left, negative for right
    gz_drive_publisher_.Publish(msg);   
}
