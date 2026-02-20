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
    std::string gz_camera_topic = "/world/" + payload_params_.sim.world_name + "/model/" + payload_name + "/link/camera_link/sensor/camera/image";
    std::string gz_camera_info_topic = "/world/" + payload_params_.sim.world_name + "/model/" + payload_name + "/link/camera_link/sensor/camera/camera_info";

    std::string ros_camera_topic = "/" + payload_name + "/camera";
    ros_camera_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(ros_camera_topic, 10);
    RCLCPP_INFO(node_->get_logger(), "Publishing ros camera: %s", ros_camera_topic.c_str());

    std::string ros_camera_info_topic = "/" + payload_name + "/camera_info";
    ros_camera_info_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(ros_camera_info_topic, 10);
    RCLCPP_INFO(node_->get_logger(), "Publishing ros camera_info: %s", ros_camera_info_topic.c_str());

    gz_drive_publisher_ = gz_node_->Advertise<gz::msgs::Twist>(gz_drive_topic);
    RCLCPP_INFO(node_->get_logger(), "SIM | Publishing gz drive commands to %s", gz_drive_topic.c_str());
    gz_node_->Subscribe(gz_camera_topic, &SimController::gz_camera_callback, this);
    RCLCPP_INFO(node_->get_logger(), "SIM | Listening to gz camera msgs from %s", gz_camera_topic.c_str());
    gz_node_->Subscribe(gz_camera_info_topic, &SimController::gz_camera_info_callback, this);
    RCLCPP_INFO(node_->get_logger(), "SIM | Listening to gz camera_info msgs from %s", gz_camera_info_topic.c_str());
}

void SimController::drive_command(double linear, double angular) {
    gz::msgs::Twist msg;
    msg.mutable_linear()->set_x(linear); //head-on direction
    msg.mutable_angular()->set_z(angular); //positive for left, negative for right
    gz_drive_publisher_.Publish(msg);
}

void SimController::gz_camera_info_callback(const gz::msgs::CameraInfo& gz_msg) {
    sensor_msgs::msg::CameraInfo ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_camera_info_publisher_->publish(ros_msg);
}

void SimController::gz_camera_callback(const gz::msgs::Image& gz_msg) {
    sensor_msgs::msg::Image ros_msg;    
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_camera_publisher_->publish(ros_msg);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(SimController, Controller)