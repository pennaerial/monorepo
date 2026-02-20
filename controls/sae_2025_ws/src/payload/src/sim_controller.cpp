#include "payload/sim_controller.hpp"


SimController::SimController() {

}

void SimController::initialize(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
    gz_node_ = std::make_shared<gz::transport::Node>();

    payload_params_listener_ = std::make_shared<payload::ParamListener>(node_);
    payload_params_ = payload_params_listener_->get_params();

    std::string payload_name = node_->get_name();
    std::string gz_drive_topic = "/model/" + payload_name + "/cmd_vel";

    gz_drive_publisher_ = gz_node_->Advertise<gz::msgs::Twist>(gz_drive_topic);
    RCLCPP_INFO(node_->get_logger(), "SIM | Publishing gz drive commands to %s", gz_drive_topic.c_str());
}

void SimController::drive_command(double linear, double angular) {
    gz::msgs::Twist msg;
    msg.mutable_linear()->set_x(linear); //head-on direction
    msg.mutable_angular()->set_z(angular); //positive for left, negative for right
    gz_drive_publisher_.Publish(msg);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(SimController, Controller)
