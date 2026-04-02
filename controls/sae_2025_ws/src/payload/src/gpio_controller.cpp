#include "payload/gpio_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <lgpio.h>
#include <pluginlib/class_list_macros.hpp>

namespace {
constexpr int kGpioChip = 0;

struct ControllerConfig {
    int loop_ms {50};
    int encoder_output_cpr {617};
    int encoder_left_sign {1};
    int encoder_right_sign {1};
    double wheel_radius_m {0.01611839};
    double wheel_separation_m {0.12132};
    double max_wheel_rpm {120.0};
    payload::control_math::PidConfig left_pid  {0.02, 0.0, 0.0, 1.0, 1.0, 1.0};
    payload::control_math::PidConfig right_pid {0.02, 0.0, 0.0, 1.0, 1.0, 1.0};
    double velocity_alpha {1.0};
};

ControllerConfig config_from_params(const payload::Params& p)
{
    ControllerConfig c;
    c.loop_ms             = p.control.loop_ms;
    c.encoder_output_cpr  = p.encoder.output_cpr;
    c.encoder_left_sign   = p.encoder.left_sign;
    c.encoder_right_sign  = p.encoder.right_sign;
    c.wheel_radius_m      = p.kinematics.wheel_radius_m;
    c.wheel_separation_m  = p.kinematics.wheel_separation_m;
    c.max_wheel_rpm       = p.motor.max_wheel_rpm;
    c.left_pid  = {p.pid.left_kp,  p.pid.left_ki,  p.pid.left_kd,
                   p.pid.i_clamp, p.pid.output_limit_norm, p.pid.stop_deadband_rpm};
    c.right_pid = {p.pid.right_kp, p.pid.right_ki, p.pid.right_kd,
                   p.pid.i_clamp, p.pid.output_limit_norm, p.pid.stop_deadband_rpm};
    c.velocity_alpha = p.pid.velocity_alpha;
    return c;
}
}  // namespace

GPIOController::GPIOController() {}

GPIOController::~GPIOController()
{
    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    if (left_motor_)  left_motor_->coast();
    if (right_motor_) right_motor_->coast();

    left_encoder_.reset();
    right_encoder_.reset();
    left_motor_.reset();
    right_motor_.reset();

    if (handle_ >= 0) {
        lgGpiochipClose(handle_);
        handle_ = -1;
    }
}

void GPIOController::initialize(std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;

    RCLCPP_INFO(node_->get_logger(), "GPIO | Starting initialization...");

    param_listener_ = std::make_shared<payload::ParamListener>(node_);
    const auto p = param_listener_->get_params();

    handle_ = lgGpiochipOpen(kGpioChip);
    if (handle_ < 0) {
        RCLCPP_FATAL(node_->get_logger(),
            "GPIO | lgGpiochipOpen(%d) failed (err=%d)", kGpioChip, handle_);
        return;
    }

    const int pwm_hz = std::max(1, static_cast<int>(std::lround(p.motor.pwm_frequency)));

    left_motor_  = std::make_unique<SNMotor>(
        handle_, p.pins.LEFT_PWM,  p.pins.LEFT_IN1,  p.pins.LEFT_IN2,  pwm_hz, MotorType::LEFT);
    right_motor_ = std::make_unique<SNMotor>(
        handle_, p.pins.RIGHT_PWM, p.pins.RIGHT_IN1, p.pins.RIGHT_IN2, pwm_hz, MotorType::RIGHT);

    left_encoder_ = std::make_unique<QuadratureEncoder>(
        handle_, p.pins.ENCB_CH1, p.pins.ENCB_CH2, p.encoder.output_cpr, MotorType::LEFT);
    right_encoder_ = std::make_unique<QuadratureEncoder>(
        handle_, p.pins.ENCA_CH1, p.pins.ENCA_CH2, p.encoder.output_cpr, MotorType::RIGHT);

    prev_left_count_  = left_encoder_->count();
    prev_right_count_ = right_encoder_->count();
    prev_loop_time_   = std::chrono::steady_clock::now();

    left_filtered_rpm_  = 0.0;
    right_filtered_rpm_ = 0.0;
    left_pid_state_     = payload::control_math::PidState{};
    right_pid_state_    = payload::control_math::PidState{};

    const std::string node_name = node_->get_name();
    const std::string state_topic = "/" + node_name + "/motor_state";
    const std::string zn_service_name = "/" + node_name + "/compute_pid_zn";

    motor_state_pub_ = node_->create_publisher<payload_interfaces::msg::MotorState>(state_topic, 10);
    zn_service_ = node_->create_service<payload_interfaces::srv::ComputePidZieglerNichols>(
        zn_service_name,
        [this](const std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Request> req,
               std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Response> res) {
            compute_pid_zn_callback(req, res);
        });

    running_ = true;
    control_thread_ = std::thread(&GPIOController::control_loop, this);

    RCLCPP_INFO(node_->get_logger(),
        "GPIO | Ready. state_topic=%s zn_service=%s",
        state_topic.c_str(), zn_service_name.c_str());
}

void GPIOController::drive_command(double linear, double angular)
{
    cmd_linear_.store(linear);
    cmd_angular_.store(angular);
}

void GPIOController::compute_pid_zn_callback(
    const std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Request> request,
    std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Response> response)
{
    payload::control_math::PidGains gains;
    std::string error;

    const bool ok = payload::control_math::compute_ziegler_nichols_classic(
        request->ku, request->pu, gains, error);

    response->success = ok;
    if (ok) {
        response->message = "OK";
        response->kp = gains.kp;
        response->ki = gains.ki;
        response->kd = gains.kd;
    } else {
        response->message = error;
        response->kp = 0.0;
        response->ki = 0.0;
        response->kd = 0.0;
    }
}

void GPIOController::control_loop()
{
    while (running_) {
        const auto loop_start = std::chrono::steady_clock::now();

        const ControllerConfig config = config_from_params(param_listener_->get_params());

        const double linear  = cmd_linear_.load();
        const double angular = cmd_angular_.load();
        const auto setpoints = payload::control_math::compute_wheel_setpoints(
            linear, angular,
            config.wheel_separation_m, config.wheel_radius_m, config.max_wheel_rpm);

        const int64_t left_count  = left_encoder_  ? left_encoder_->count()  : 0;
        const int64_t right_count = right_encoder_ ? right_encoder_->count() : 0;

        const auto now = std::chrono::steady_clock::now();
        double dt_s = std::chrono::duration<double>(now - prev_loop_time_).count();
        if (dt_s <= 0.0) {
            dt_s = static_cast<double>(config.loop_ms) / 1000.0;
        }
        prev_loop_time_ = now;

        const int64_t left_delta =
            (left_count - prev_left_count_) * static_cast<int64_t>(config.encoder_left_sign);
        const int64_t right_delta =
            (right_count - prev_right_count_) * static_cast<int64_t>(config.encoder_right_sign);

        prev_left_count_  = left_count;
        prev_right_count_ = right_count;

        const double left_raw_rpm = payload::control_math::rpm_from_count_delta(
            left_delta, config.encoder_output_cpr, dt_s);
        const double right_raw_rpm = payload::control_math::rpm_from_count_delta(
            right_delta, config.encoder_output_cpr, dt_s);

        left_filtered_rpm_ = payload::control_math::low_pass_filter(
            left_raw_rpm, left_filtered_rpm_, config.velocity_alpha);
        right_filtered_rpm_ = payload::control_math::low_pass_filter(
            right_raw_rpm, right_filtered_rpm_, config.velocity_alpha);

        const auto left_terms = payload::control_math::pid_step(
            setpoints.left_rpm, left_filtered_rpm_, dt_s, config.left_pid, left_pid_state_);
        const auto right_terms = payload::control_math::pid_step(
            setpoints.right_rpm, right_filtered_rpm_, dt_s, config.right_pid, right_pid_state_);

        left_motor_->set_speed(static_cast<float>(left_terms.output));
        right_motor_->set_speed(static_cast<float>(right_terms.output));

        if (motor_state_pub_) {
            const float left_pwm_percent  = static_cast<float>(
                std::clamp(std::abs(left_terms.output)  * 100.0, 0.0, 100.0));
            const float right_pwm_percent = static_cast<float>(
                std::clamp(std::abs(right_terms.output) * 100.0, 0.0, 100.0));

            payload_interfaces::msg::MotorState msg;
            msg.linear_setpoint_mps    = linear;
            msg.angular_setpoint_rad_s = angular;
            msg.left_setpoint_rpm      = setpoints.left_rpm;
            msg.right_setpoint_rpm     = setpoints.right_rpm;
            msg.left_measured_rpm      = left_filtered_rpm_;
            msg.right_measured_rpm     = right_filtered_rpm_;
            msg.left_pid_error_rpm     = left_terms.error;
            msg.right_pid_error_rpm    = right_terms.error;
            msg.left_pid_p             = left_terms.p;
            msg.left_pid_i             = left_terms.i;
            msg.left_pid_d             = left_terms.d;
            msg.right_pid_p            = right_terms.p;
            msg.right_pid_i            = right_terms.i;
            msg.right_pid_d            = right_terms.d;
            msg.left_output_norm       = left_terms.output;
            msg.right_output_norm      = right_terms.output;
            msg.left_pwm_percent       = left_pwm_percent;
            msg.right_pwm_percent      = right_pwm_percent;
            msg.left_encoder_count     = left_count;
            msg.right_encoder_count    = right_count;
            motor_state_pub_->publish(msg);
        }

        std::this_thread::sleep_until(
            loop_start + std::chrono::milliseconds(config.loop_ms));
    }
}

PLUGINLIB_EXPORT_CLASS(GPIOController, Controller)
