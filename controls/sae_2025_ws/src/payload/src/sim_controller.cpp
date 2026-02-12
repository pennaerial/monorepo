#include "payload/sim_controller.hpp"

SimController::SimController(
    const std::string& gz_drive_topic, 
    const std::string& gz_camera_topic, 
    const std::string& gz_camera_info_topic, 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_camera_publisher,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_camera_info_publisher,
    rclcpp::Logger logger)
: logger_(logger) {
    logger_ = logger;
    gz_node_ = std::make_shared<gz::transport::Node>();

    gz_drive_publisher_ = gz_node_->Advertise<gz::msgs::Twist>(gz_drive_topic);
    RCLCPP_INFO(logger_, "SIM | Publishing gz drive commands to %s", gz_drive_topic.c_str());
    gz_node_->Subscribe(gz_camera_topic, &SimController::gz_camera_callback, this);
    RCLCPP_INFO(logger_, "SIM | Listening to gz camera msgs from %s", gz_camera_topic.c_str());
    gz_node_->Subscribe(gz_camera_info_topic, &SimController::gz_camera_info_callback, this);
    RCLCPP_INFO(logger_, "SIM | Listening to gz camera_info msgs from %s", gz_camera_info_topic.c_str());

    ros_camera_publisher_ = ros_camera_publisher;
    ros_camera_info_publisher_ = ros_camera_info_publisher;

}

void SimController::drive_command(double linear, double angular) {
    static int publish_count = 0;
    if (++publish_count % 30 == 1) {
        RCLCPP_INFO(
            logger_,
            "DEBUG | SimController publishing to gz: linear=%.2f, angular=%.2f (count=%d)",
            linear, angular, publish_count);
    }
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