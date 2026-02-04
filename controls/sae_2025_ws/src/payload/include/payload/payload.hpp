#ifndef PAYLOAD_HPP
#define PAYLOAD_HPP

#include <rclcpp/rclcpp.hpp>
#include "payload_interfaces/msg/drive_command.hpp"
#include "payload/controller.hpp"
#include <string>

class Payload: public rclcpp::Node {
    public:
        Payload(const std::string& payload_name);

    private:
        std::string payload_name_;
        rclcpp::Subscription<payload_interfaces::msg::DriveCommand>::SharedPtr drive_subscriber_;
        std::shared_ptr<Controller> controller_backend_;
        void drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg);
};

#endif