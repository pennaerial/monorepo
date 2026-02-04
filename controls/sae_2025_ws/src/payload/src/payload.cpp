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

    std::string ros_camera_topic = "/" + payload_name_ + "/camera";
    ros_camera_publisher_ =  this->create_publisher<sensor_msgs::msg::Image>(ros_camera_topic, 10);
    RCLCPP_INFO(this->get_logger(), "Publishing ros camera: %s", ros_camera_topic.c_str());

    std::string ros_camera_info_topic = "/" + payload_name_ + "/camera_info";
    ros_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_camera_info_topic, 10);
    RCLCPP_INFO(this->get_logger(), "Publishing ros camera_info: %s", ros_camera_info_topic.c_str());

    if (payload_params_.controller == "sim") { //move to enum
        std::string gz_drive_topic = "/model/" + payload_name_ + "/cmd_vel";
        std::string gz_camera_topic = "/world/" + payload_params_.sim.world_name + "/model/" + payload_name_ + "/link/camera_link/sensor/camera/image";
        std::string gz_camera_info_topic = "/world/" + payload_params_.sim.world_name + "/model/" + payload_name_ + "/link/camera_link/sensor/camera/camera_info";
        controller_ = std::make_shared<SimController>(
            gz_drive_topic,
            gz_camera_topic,
            gz_camera_info_topic,
            ros_camera_publisher_,
            ros_camera_info_publisher_,
            this->get_logger()
        );
    }
}

void Payload::drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg) {
    double linear_v = msg->linear;
    double angular_v = msg->angular;
    controller_->drive_command(linear_v, angular_v);
}



