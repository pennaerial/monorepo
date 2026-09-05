#include "payload_controller/payload_controller.hpp"

#include <functional>
#include <stdexcept>
#include <utility>

Payload::Payload(const std::string& vehicle_name)
    : rclcpp::Node(vehicle_name), controller_loader_("payload_controller", "Controller")
{
  create_command_subscriptions();
  load_controller_from_params();
}

Payload::Payload(const std::string& vehicle_name, std::shared_ptr<Controller> controller)
    : rclcpp::Node(vehicle_name),
      controller_loader_("payload_controller", "Controller"),
      controller_(std::move(controller)),
      controller_name_("<injected>")
{
  if (!controller_) {
    throw std::invalid_argument("Injected payload controller must not be null.");
  }
  create_command_subscriptions();
  RCLCPP_INFO(this->get_logger(), "Using injected payload controller for testing.");
}

void Payload::create_command_subscriptions()
{
  std::string ros_drive_topic = "cmd_drive";
  ros_drive_subscriber_ = this->create_subscription<payload_interfaces::msg::DriveCommand>(
      ros_drive_topic, 10, std::bind(&Payload::drive_callback, this, std::placeholders::_1)
  );
  RCLCPP_INFO(this->get_logger(), "Listening on: %s", ros_drive_topic.c_str());

  std::string servo_topic = "servo";
  servo_subscriber_ = this->create_subscription<payload_interfaces::msg::ServoCommand>(
      servo_topic, 10, std::bind(&Payload::servo_callback, this, std::placeholders::_1)
  );
  RCLCPP_INFO(this->get_logger(), "Servo listening on: %s", servo_topic.c_str());
}

void Payload::load_controller_from_params()
{
  payload_params_listener_ = std::make_shared<payload::ParamListener>(this);
  const auto params = payload_params_listener_->get_params();
  controller_name_ = params.controller;

  RCLCPP_INFO(this->get_logger(), "Loading payload controller plugin: %s", controller_name_.c_str());
  try {
    controller_ = controller_loader_.createSharedInstance(controller_name_);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(
        this->get_logger(), "Failed to create payload controller '%s': %s", controller_name_.c_str(), ex.what()
    );
    throw;
  }
}

Payload::~Payload()
{
  prepare_for_shutdown();
  timed_drive_srv_.reset();
  ros_drive_subscriber_.reset();
  servo_subscriber_.reset();
  payload_params_listener_.reset();
  controller_name_.clear();
  controller_.reset();
}

void Payload::prepare_for_shutdown() noexcept
{
  shutdown_actuators();
}

void Payload::shutdown_actuators() noexcept
{
  if (shutdown_actuators_requested_.exchange(true)) {
    return;
  }

  if (timed_drive_timer_) {
    timed_drive_timer_->cancel();
    timed_drive_timer_.reset();
  }
  timed_override_active_.store(false);

  if (controller_) {
    try {
      controller_->safe_shutdown();
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(
          this->get_logger(), "Failed to safely shut down payload controller '%s': %s", controller_name_.c_str(),
          ex.what()
      );
    } catch (...) {
      RCLCPP_ERROR(
          this->get_logger(), "Failed to safely shut down payload controller '%s' due to an unknown error.",
          controller_name_.c_str()
      );
    }
  }
}

void Payload::init()
{
  try {
    controller_->initialize(this);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(
        this->get_logger(), "Failed to initialize payload controller '%s': %s", controller_name_.c_str(), ex.what()
    );
    throw;
  }
  RCLCPP_INFO(this->get_logger(), "Payload controller initialized: %s", controller_name_.c_str());

  std::string timed_drive_name = "timed_drive";
  timed_drive_srv_ = this->create_service<payload_interfaces::srv::TimedDrive>(
      timed_drive_name, std::bind(&Payload::timed_drive_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  RCLCPP_INFO(this->get_logger(), "Timed drive service: %s", timed_drive_name.c_str());
}

void Payload::drive_callback(const payload_interfaces::msg::DriveCommand::SharedPtr msg)
{
  if (shutdown_actuators_requested_.load() || timed_override_active_.load() || !controller_) {
    return;
  }
  controller_->drive_command(msg->linear, msg->angular);
}

void Payload::servo_callback(const payload_interfaces::msg::ServoCommand::SharedPtr msg)
{
  if (shutdown_actuators_requested_.load() || !controller_) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Servo command: %.2f deg", msg->degree);
  controller_->servo_command(msg->degree);
}

void Payload::clear_timed_override()
{
  if (timed_drive_timer_) {
    timed_drive_timer_->cancel();
    timed_drive_timer_.reset();
  }
  timed_override_active_.store(false);
  if (controller_) {
    controller_->drive_command(0.0, 0.0);
  }
}

void Payload::timed_drive_callback(
    const std::shared_ptr<payload_interfaces::srv::TimedDrive::Request> request,
    std::shared_ptr<payload_interfaces::srv::TimedDrive::Response> response
)
{
  if (shutdown_actuators_requested_.load()) {
    response->success = false;
    response->message = "payload is shutting down";
    return;
  }
  if (request->duration_sec <= 0.0) {
    response->success = false;
    response->message = "duration_sec must be > 0";
    return;
  }
  if (timed_drive_timer_) {
    timed_drive_timer_->cancel();
    timed_drive_timer_.reset();
  }
  timed_override_active_.store(true);
  if (controller_) {
    controller_->drive_command(request->linear, request->angular);
  }
  timed_drive_timer_ = this->create_wall_timer(std::chrono::duration<double>(request->duration_sec), [this]() {
    clear_timed_override();
  });
  response->success = true;
  response->message = "Timed drive started";
}
