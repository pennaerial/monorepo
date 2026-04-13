#ifndef PAYLOAD_HPP
#define PAYLOAD_HPP

#include <atomic>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "payload_interfaces/msg/drive_command.hpp"
#include "payload_interfaces/msg/servo_command.hpp"
#include "payload_interfaces/srv/timed_drive.hpp"
#include "payload/payload_parameters.hpp"
#include "payload/controller.hpp"

class Payload : public rclcpp::Node
{
public:
  Payload(const std::string & vehicle_name);
  Payload(const std::string & vehicle_name, std::shared_ptr<Controller> controller);
  ~Payload() override;
  void init();
  void prepare_for_shutdown() noexcept;

private:
  rclcpp::Subscription<payload_interfaces::msg::DriveCommand>::SharedPtr ros_drive_subscriber_;
  rclcpp::Subscription<payload_interfaces::msg::ServoCommand>::SharedPtr servo_subscriber_;

  pluginlib::ClassLoader<Controller> controller_loader_;
  std::shared_ptr<Controller> controller_;

  std::shared_ptr<payload::ParamListener> payload_params_listener_;
  std::string controller_name_;

  std::atomic<bool> timed_override_active_{false};
  std::atomic<bool> shutdown_actuators_requested_{false};
  rclcpp::TimerBase::SharedPtr timed_drive_timer_;
  rclcpp::Service<payload_interfaces::srv::TimedDrive>::SharedPtr timed_drive_srv_;

  void create_command_subscriptions();
  void load_controller_from_params();
  void drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg);
  void servo_callback(const payload_interfaces::msg::ServoCommand::SharedPtr msg);
  void timed_drive_callback(
    const std::shared_ptr<payload_interfaces::srv::TimedDrive::Request> request,
    std::shared_ptr<payload_interfaces::srv::TimedDrive::Response> response);
  void clear_timed_override();
  void shutdown_actuators() noexcept;
};

#endif
