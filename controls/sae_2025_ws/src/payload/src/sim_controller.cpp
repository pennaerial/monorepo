#include "payload/sim_controller.hpp"

SimController::SimController(
    const std::string& gz_drive_topic,
    const std::string& gz_camera_topic,
    const std::string& gz_camera_info_topic,
    // const std::string& gz_pose_topic,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_camera_publisher,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_camera_info_publisher,
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_pose_global_publisher,
    // std::function<rclcpp::Time()> get_clock_now,
    rclcpp::Logger logger)
: logger_(logger) {
    gz_node_ = std::make_shared<gz::transport::Node>();

    gz_drive_publisher_ = gz_node_->Advertise<gz::msgs::Twist>(gz_drive_topic);
    gz_node_->Subscribe(gz_camera_topic, &SimController::gz_camera_callback, this);
    gz_node_->Subscribe(gz_camera_info_topic, &SimController::gz_camera_info_callback, this);
    // gz_node_->Subscribe(gz_pose_topic, &SimController::gz_pose_callback, this);

    ros_camera_publisher_ = ros_camera_publisher;
    ros_camera_info_publisher_ = ros_camera_info_publisher;
    // ros_pose_global_publisher_ = ros_pose_global_publisher;
    // get_clock_now_ = std::move(get_clock_now);
}

void SimController::drive_command(double linear, double angular) {
    gz::msgs::Twist msg;
    msg.mutable_linear()->set_x(linear);
    msg.mutable_angular()->set_z(angular);
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

// void SimController::gz_pose_callback(const gz::msgs::Pose& gz_msg) {
//     geometry_msgs::msg::PoseStamped ros_msg;
//     ros_msg.header.frame_id = "world";
//     ros_msg.header.stamp = get_clock_now_();
//     ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg.pose);
//     ros_pose_global_publisher_->publish(ros_msg);
// }