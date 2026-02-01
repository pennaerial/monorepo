#ifndef PAYLOAD_CONTROLLER_HPP
#define PAYLOAD_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "payload_interfaces/msg/drive_command.hpp"
#include "payload/controller_backend.hpp"
#include <string>

class PayloadController: public rclcpp::Node {
    public:
        PayloadController(const std::string& payload_name);

    private:
        std::string payload_name_;
        rclcpp::Subscription<payload_interfaces::msg::DriveCommand>::SharedPtr drive_subscriber_;
        std::shared_ptr<ControllerBackend> controller_backend_;
        void drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg);

        

};

#endif