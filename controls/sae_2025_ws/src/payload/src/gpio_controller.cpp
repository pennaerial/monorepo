#include "payload/gpio_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace {
constexpr int kGpioChip = 0;
}  // namespace

GPIOController::GPIOController() {}

GPIOController::~GPIOController()
{
    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    ControllerConfig config;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config = config_;
    }

    left_encoder_.reset();
    right_encoder_.reset();

    if (handle_ >= 0) {
        lgTxPwm(handle_, config.left_pwm_pin, 0, 0.0f, 0, 0);
        lgTxPwm(handle_, config.right_pwm_pin, 0, 0.0f, 0, 0);
        lgGpioWrite(handle_, config.left_in1_pin, 0);
        lgGpioWrite(handle_, config.left_in2_pin, 0);
        lgGpioWrite(handle_, config.right_in1_pin, 0);
        lgGpioWrite(handle_, config.right_in2_pin, 0);
        lgGpiochipClose(handle_);
        handle_ = -1;
    }
}

void GPIOController::initialize(rclcpp::Node* node)
{
    RCLCPP_INFO(node->get_logger(), "GPIO | Starting initialization...");

    load_initial_config_from_parameters();

    handle_ = lgGpiochipOpen(kGpioChip);
    if (handle_ < 0) {
        RCLCPP_FATAL(node->get_logger(),
            "GPIO | lgGpiochipOpen(%d) failed (err=%d)",
            kGpioChip, handle_);
        return;
    }

    ControllerConfig config;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config = config_;
    }

    RCLCPP_INFO(node->get_logger(), "LEFT PINS: PWM: %d, IN1: %d, IN2: %d", config.left_pwm_pin, config.left_in1_pin, config.left_in2_pin);
    RCLCPP_INFO(node->get_logger(), "RIGHT PINS: PWM: %d, IN1: %d, IN2: %d", config.right_pwm_pin, config.right_in1_pin, config.right_in2_pin);
    const int rc_left_in1 = lgGpioClaimOutput(handle_, 0, config.left_in1_pin, 0);
    const int rc_left_in2 = lgGpioClaimOutput(handle_, 0, config.left_in2_pin, 0);
    const int rc_right_in1 = lgGpioClaimOutput(handle_, 0, config.right_in1_pin, 0);
    const int rc_right_in2 = lgGpioClaimOutput(handle_, 0, config.right_in2_pin, 0);
    const int rc_left_pwm = lgGpioClaimOutput(handle_, 0, config.left_pwm_pin, 0);
    const int rc_right_pwm = lgGpioClaimOutput(handle_, 0, config.right_pwm_pin, 0);

    if (rc_left_in1 < 0 || rc_left_in2 < 0 ||
        rc_right_in1 < 0 || rc_right_in2 < 0 ||
        rc_left_pwm < 0 || rc_right_pwm < 0) {
        RCLCPP_FATAL(node->get_logger(),
            "GPIO | Failed claiming motor pins: LIN1=%d LIN2=%d RIN1=%d RIN2=%d LPWM=%d RPWM=%d",
            rc_left_in1, rc_left_in2, rc_right_in1, rc_right_in2, rc_left_pwm, rc_right_pwm);
        return;
    }

    const int rc_pwm_l = lgTxPwm(handle_, config.left_pwm_pin, config.pwm_hz, 0.0f, 0, 0);
    const int rc_pwm_r = lgTxPwm(handle_, config.right_pwm_pin, config.pwm_hz, 0.0f, 0, 0);
    if (rc_pwm_l < 0 || rc_pwm_r < 0) {
        RCLCPP_FATAL(node->get_logger(),
            "GPIO | Failed starting PWM: left_rc=%d right_rc=%d", rc_pwm_l, rc_pwm_r);
        return;
    }

    left_encoder_ = std::make_unique<QuadratureEncoder>(
        handle_, config.enc_left_a, config.enc_left_b, config.encoder_output_cpr, MotorType::LEFT);
    right_encoder_ = std::make_unique<QuadratureEncoder>(
        handle_, config.enc_right_a, config.enc_right_b, config.encoder_output_cpr, MotorType::RIGHT);

    prev_left_count_ = left_encoder_->count();
    prev_right_count_ = right_encoder_->count();
    prev_loop_time_ = std::chrono::steady_clock::now();

    left_filtered_rpm_ = 0.0;
    right_filtered_rpm_ = 0.0;
    left_pid_state_ = payload::control_math::PidState{};
    right_pid_state_ = payload::control_math::PidState{};

    const std::string node_name = node->get_name();
    const std::string state_topic = "/" + node_name + "/motor_state";
    const std::string zn_service_name = "/" + node_name + "/compute_pid_zn";

    motor_state_pub_ = node->create_publisher<payload_interfaces::msg::MotorState>(state_topic, 10);
    zn_service_ = node->create_service<payload_interfaces::srv::ComputePidZieglerNichols>(
        zn_service_name,
        std::bind(&GPIOController::compute_pid_zn_callback, this, std::placeholders::_1, std::placeholders::_2));

    parameter_callback_handle_ = node->add_on_set_parameters_callback(
        std::bind(&GPIOController::on_parameters_set, this, std::placeholders::_1));

    running_ = true;
    control_thread_ = std::thread(&GPIOController::control_loop, this);

    RCLCPP_INFO(node->get_logger(),
        "GPIO | Ready. state_topic=%s zn_service=%s",
        state_topic.c_str(), zn_service_name.c_str());
}

void GPIOController::drive_command(double linear, double angular) {
    
}
