#include "payload/payload.hpp"

Payload::Payload(const std::string& payload_name)
: rclcpp::Node(payload_name),
  controller_loader_("payload", "Controller") {
    payload_name_ = this->get_name();

    payload_params_listener_ = std::make_shared<payload::ParamListener>(this);
    payload_params_ = payload_params_listener_->get_params();

    const std::string ros_drive_topic = "/" + payload_name_ + "/cmd_drive";
    ros_drive_subscriber_ =
        this->create_subscription<payload_interfaces::msg::DriveCommand>(
            ros_drive_topic,
            10,
            std::bind(&Payload::drive_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", ros_drive_topic.c_str());

    const std::string servo_topic = "/" + payload_name_ + "/servo";
    servo_subscriber_ =
        this->create_subscription<payload_interfaces::msg::ServoCommand>(
            servo_topic,
            10,
            std::bind(&Payload::servo_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Servo listening on: %s", servo_topic.c_str());

    controller_ = controller_loader_.createSharedInstance(payload_params_.controller);
}

Payload::~Payload() {
    if (timed_drive_timer_) {
        timed_drive_timer_->cancel();
        timed_drive_timer_.reset();
    }
    timed_override_active_.store(false);
    timed_drive_srv_.reset();
    ros_drive_subscriber_.reset();
    servo_subscriber_.reset();
    if (controller_) {
        controller_->drive_command(0.0, 0.0);
        controller_.reset();
    }
}

void Payload::init() {
    controller_->initialize(this);

    const std::string timed_drive_name = "/" + payload_name_ + "/timed_drive";
    timed_drive_srv_ = this->create_service<payload_interfaces::srv::TimedDrive>(
        timed_drive_name,
        std::bind(&Payload::timed_drive_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
        this->get_logger(), "Timed drive service: %s", timed_drive_name.c_str());
}

void Payload::drive_callback(
    const payload_interfaces::msg::DriveCommand::SharedPtr msg) {
    if (timed_override_active_.load()) {
        return;
    }
    controller_->drive_command(msg->linear, msg->angular);
}

void Payload::clear_timed_override() {
    if (timed_drive_timer_) {
        timed_drive_timer_->cancel();
        timed_drive_timer_.reset();
    }
    controller_->drive_command(0.0, 0.0);
    timed_override_active_.store(false);
}

void Payload::timed_drive_callback(
    const std::shared_ptr<payload_interfaces::srv::TimedDrive::Request> request,
    std::shared_ptr<payload_interfaces::srv::TimedDrive::Response> response) {
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
    controller_->drive_command(request->linear, request->angular);
    timed_drive_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(request->duration_sec),
        [this]() { clear_timed_override(); });
    response->success = true;
    response->message = "Timed drive started";
}

void Payload::servo_callback(
    const payload_interfaces::msg::ServoCommand::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Servo command: %.2f deg", msg->degree);
    controller_->servo_command(msg->degree);
}
