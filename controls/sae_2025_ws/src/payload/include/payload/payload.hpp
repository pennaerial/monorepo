#ifndef PAYLOAD_HPP
#define PAYLOAD_HPP

#include <atomic>
#include <chrono>
#include <payload_interfaces/msg/detail/servo_command__struct.hpp>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "payload_interfaces/msg/drive_command.hpp"
#include "payload_interfaces/msg/servo_command.hpp"
#include "payload_interfaces/srv/timed_drive.hpp"
#include "payload/payload_parameters.hpp"
#include "payload/controller.hpp"

class Payload {
public:
    explicit Payload(std::shared_ptr<rclcpp::Node> node);
    void init();

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<payload_interfaces::msg::DriveCommand>::SharedPtr ros_drive_subscriber_;

    pluginlib::ClassLoader<Controller> controller_loader_;
    std::shared_ptr<Controller> controller_;

    std::shared_ptr<payload::ParamListener> payload_params_listener_;
    payload::Params payload_params_;

    rclcpp::Subscription<payload_interfaces::msg::ServoCommand>::SharedPtr servo_subscriber_;

    std::atomic<bool> timed_override_active_{false};
    rclcpp::TimerBase::SharedPtr timed_drive_timer_;
    rclcpp::Service<payload_interfaces::srv::TimedDrive>::SharedPtr timed_drive_srv_;


    void drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg);
    void timed_drive_callback(
        const std::shared_ptr<payload_interfaces::srv::TimedDrive::Request> request,
        std::shared_ptr<payload_interfaces::srv::TimedDrive::Response> response);
    void clear_timed_override();
    void servo_callback(const payload_interfaces::msg::ServoCommand::SharedPtr msg);
};

#endif
