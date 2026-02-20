#ifndef PAYLOAD_HPP
#define PAYLOAD_HPP

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include "payload_interfaces/msg/drive_command.hpp"
#include "payload/payload_parameters.hpp"
#include "payload/controller.hpp"
#include <string>


class Payload : public rclcpp::Node {
    public:
        Payload(const std::string& payload_name);
        void init();

    private:
        std::string payload_name_;
        rclcpp::Subscription<payload_interfaces::msg::DriveCommand>::SharedPtr ros_drive_subscriber_;

        pluginlib::ClassLoader<Controller> controller_loader_;
        std::shared_ptr<Controller> controller_;

        std::shared_ptr<payload::ParamListener> payload_params_listener_;
        payload::Params payload_params_;

        void drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg);

};

#endif