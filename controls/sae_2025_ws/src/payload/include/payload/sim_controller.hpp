#ifndef SIM_CONTROLLER_HPP
#define SIM_CONTROLLER_HPP


#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>
// #include <gz/msgs/pose.pb.h>
#include "ros_gz_bridge/convert.hpp"
#include "payload/payload.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
// #include <functional>

class SimController : public Controller {
    public:
        SimController(
            const std::string& gz_drive_topic,
            const std::string& gz_camera_topic,
            const std::string& gz_camera_info_topic,
            // const std::string& gz_pose_topic,
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_camera_publisher,
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_camera_info_publisher,
            // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_pose_global_publisher,
            // std::function<rclcpp::Time()> get_clock_now,
            rclcpp::Logger logger);
        void drive_command(double linear, double angular) override;

    private:
        std::shared_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher gz_drive_publisher_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_camera_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_camera_info_publisher_;
        // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_pose_global_publisher_;
        // std::function<rclcpp::Time()> get_clock_now_;
        rclcpp::Logger logger_;

        void gz_camera_callback(const gz::msgs::Image& gz_msg);
        void gz_camera_info_callback(const gz::msgs::CameraInfo& gz_msg);
        // void gz_pose_callback(const gz::msgs::Pose& gz_msg);
};

#endif