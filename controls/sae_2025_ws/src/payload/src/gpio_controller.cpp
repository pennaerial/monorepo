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

void GPIOController::initialize(std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;

    RCLCPP_INFO(node_->get_logger(), "GPIO | Starting initialization...");

    load_initial_config_from_parameters();

    handle_ = lgGpiochipOpen(kGpioChip);
    if (handle_ < 0) {
        RCLCPP_FATAL(node_->get_logger(),
            "GPIO | lgGpiochipOpen(%d) failed (err=%d)",
            kGpioChip, handle_);
        return;
    }

    ControllerConfig config;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config = config_;
    }

    const int rc_left_in1 = lgGpioClaimOutput(handle_, 0, config.left_in1_pin, 0);
    const int rc_left_in2 = lgGpioClaimOutput(handle_, 0, config.left_in2_pin, 0);
    const int rc_right_in1 = lgGpioClaimOutput(handle_, 0, config.right_in1_pin, 0);
    const int rc_right_in2 = lgGpioClaimOutput(handle_, 0, config.right_in2_pin, 0);
    const int rc_left_pwm = lgGpioClaimOutput(handle_, 0, config.left_pwm_pin, 0);
    const int rc_right_pwm = lgGpioClaimOutput(handle_, 0, config.right_pwm_pin, 0);

    if (rc_left_in1 < 0 || rc_left_in2 < 0 ||
        rc_right_in1 < 0 || rc_right_in2 < 0 ||
        rc_left_pwm < 0 || rc_right_pwm < 0) {
        RCLCPP_FATAL(node_->get_logger(),
            "GPIO | Failed claiming motor pins: LIN1=%d LIN2=%d RIN1=%d RIN2=%d LPWM=%d RPWM=%d",
            rc_left_in1, rc_left_in2, rc_right_in1, rc_right_in2, rc_left_pwm, rc_right_pwm);
        return;
    }

    const int rc_pwm_l = lgTxPwm(handle_, config.left_pwm_pin, config.pwm_hz, 0.0f, 0, 0);
    const int rc_pwm_r = lgTxPwm(handle_, config.right_pwm_pin, config.pwm_hz, 0.0f, 0, 0);
    if (rc_pwm_l < 0 || rc_pwm_r < 0) {
        RCLCPP_FATAL(node_->get_logger(),
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

    const std::string node_name = node_->get_name();
    const std::string state_topic = "/" + node_name + "/motor_state";
    const std::string zn_service_name = "/" + node_name + "/compute_pid_zn";

    motor_state_pub_ = node_->create_publisher<payload_interfaces::msg::MotorState>(state_topic, 10);
    zn_service_ = node_->create_service<payload_interfaces::srv::ComputePidZieglerNichols>(
        zn_service_name,
        std::bind(&GPIOController::compute_pid_zn_callback, this, std::placeholders::_1, std::placeholders::_2));

    parameter_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&GPIOController::on_parameters_set, this, std::placeholders::_1));

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

void GPIOController::load_initial_config_from_parameters()
{
    ControllerConfig config;

    // Preferred explicit left/right pin naming.
    if (!node_->get_parameter("pins.LEFT_PWM", config.left_pwm_pin)) {
        // Backwards compatibility with previous naming.
        if (!node_->get_parameter("pins.BPWM", config.left_pwm_pin)) {
            (void)node_->get_parameter("pins.BENABLE", config.left_pwm_pin);
        }
    }
    if (!node_->get_parameter("pins.RIGHT_PWM", config.right_pwm_pin)) {
        if (!node_->get_parameter("pins.APWM", config.right_pwm_pin)) {
            (void)node_->get_parameter("pins.AENABLE", config.right_pwm_pin);
        }
    }
    if (!node_->get_parameter("pins.LEFT_IN1", config.left_in1_pin)) {
        if (!node_->get_parameter("pins.BIN1", config.left_in1_pin)) {
            (void)node_->get_parameter("pins.BPHASE", config.left_in1_pin);
        }
    }
    if (!node_->get_parameter("pins.LEFT_IN2", config.left_in2_pin)) {
        (void)node_->get_parameter("pins.BIN2", config.left_in2_pin);
    }
    if (!node_->get_parameter("pins.RIGHT_IN1", config.right_in1_pin)) {
        if (!node_->get_parameter("pins.AIN1", config.right_in1_pin)) {
            (void)node_->get_parameter("pins.APHASE", config.right_in1_pin);
        }
    }
    if (!node_->get_parameter("pins.RIGHT_IN2", config.right_in2_pin)) {
        (void)node_->get_parameter("pins.AIN2", config.right_in2_pin);
    }

    (void)node_->get_parameter("pins.ENCA_CH1", config.enc_right_a);
    (void)node_->get_parameter("pins.ENCA_CH2", config.enc_right_b);
    (void)node_->get_parameter("pins.ENCB_CH1", config.enc_left_a);
    (void)node_->get_parameter("pins.ENCB_CH2", config.enc_left_b);

    double pwm_frequency = static_cast<double>(config.pwm_hz);
    (void)node_->get_parameter("motor.pwm_frequency", pwm_frequency);
    config.pwm_hz = std::max(1, static_cast<int>(std::lround(pwm_frequency)));

    (void)node_->get_parameter("motor.max_wheel_rpm", config.max_wheel_rpm);

    (void)node_->get_parameter("encoder.output_cpr", config.encoder_output_cpr);
    (void)node_->get_parameter("encoder.left_sign", config.encoder_left_sign);
    (void)node_->get_parameter("encoder.right_sign", config.encoder_right_sign);

    (void)node_->get_parameter("kinematics.wheel_radius_m", config.wheel_radius_m);
    (void)node_->get_parameter("kinematics.wheel_separation_m", config.wheel_separation_m);

    double shared_kp = config.left_pid.kp;
    double shared_ki = config.left_pid.ki;
    double shared_kd = config.left_pid.kd;
    (void)node_->get_parameter("pid.kp", shared_kp);
    (void)node_->get_parameter("pid.ki", shared_ki);
    (void)node_->get_parameter("pid.kd", shared_kd);

    config.left_pid.kp = shared_kp;
    config.right_pid.kp = shared_kp;
    config.left_pid.ki = shared_ki;
    config.right_pid.ki = shared_ki;
    config.left_pid.kd = shared_kd;
    config.right_pid.kd = shared_kd;

    (void)node_->get_parameter("pid.left_kp", config.left_pid.kp);
    (void)node_->get_parameter("pid.left_ki", config.left_pid.ki);
    (void)node_->get_parameter("pid.left_kd", config.left_pid.kd);
    (void)node_->get_parameter("pid.right_kp", config.right_pid.kp);
    (void)node_->get_parameter("pid.right_ki", config.right_pid.ki);
    (void)node_->get_parameter("pid.right_kd", config.right_pid.kd);

    (void)node_->get_parameter("pid.i_clamp", config.left_pid.i_clamp);
    config.right_pid.i_clamp = config.left_pid.i_clamp;
    (void)node_->get_parameter("pid.output_limit_norm", config.left_pid.output_limit_norm);
    config.right_pid.output_limit_norm = config.left_pid.output_limit_norm;
    (void)node_->get_parameter("pid.stop_deadband_rpm", config.left_pid.stop_deadband_rpm);
    config.right_pid.stop_deadband_rpm = config.left_pid.stop_deadband_rpm;
    (void)node_->get_parameter("pid.velocity_alpha", config.velocity_alpha);

    (void)node_->get_parameter("control.loop_ms", config.loop_ms);

    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = config;
    }
}

rcl_interfaces::msg::SetParametersResult GPIOController::on_parameters_set(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    ControllerConfig updated;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        updated = config_;
    }

    auto fail = [&](const std::string& reason) {
        result.successful = false;
        result.reason = reason;
        return result;
    };

    for (const auto& param : params) {
        const auto& name = param.get_name();
        try {
            if (name == "motor.pwm_frequency") {
                const double v = param.as_double();
                if (v <= 0.0) {
                    return fail("motor.pwm_frequency must be > 0");
                }
                updated.pwm_hz = static_cast<int>(std::lround(v));
            } else if (name == "motor.max_wheel_rpm") {
                const double v = param.as_double();
                if (v <= 0.0) {
                    return fail("motor.max_wheel_rpm must be > 0");
                }
                updated.max_wheel_rpm = v;
            } else if (name == "encoder.output_cpr") {
                const int v = param.as_int();
                if (v <= 0) {
                    return fail("encoder.output_cpr must be > 0");
                }
                updated.encoder_output_cpr = v;
            } else if (name == "encoder.left_sign") {
                const int v = param.as_int();
                if (v != -1 && v != 1) {
                    return fail("encoder.left_sign must be -1 or 1");
                }
                updated.encoder_left_sign = v;
            } else if (name == "encoder.right_sign") {
                const int v = param.as_int();
                if (v != -1 && v != 1) {
                    return fail("encoder.right_sign must be -1 or 1");
                }
                updated.encoder_right_sign = v;
            } else if (name == "kinematics.wheel_radius_m") {
                const double v = param.as_double();
                if (v <= 0.0) {
                    return fail("kinematics.wheel_radius_m must be > 0");
                }
                updated.wheel_radius_m = v;
            } else if (name == "kinematics.wheel_separation_m") {
                const double v = param.as_double();
                if (v <= 0.0) {
                    return fail("kinematics.wheel_separation_m must be > 0");
                }
                updated.wheel_separation_m = v;
            } else if (name == "pid.kp") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.kp must be >= 0");
                }
                updated.left_pid.kp = v;
                updated.right_pid.kp = v;
            } else if (name == "pid.ki") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.ki must be >= 0");
                }
                updated.left_pid.ki = v;
                updated.right_pid.ki = v;
            } else if (name == "pid.kd") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.kd must be >= 0");
                }
                updated.left_pid.kd = v;
                updated.right_pid.kd = v;
            } else if (name == "pid.left_kp") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.left_kp must be >= 0");
                }
                updated.left_pid.kp = v;
            } else if (name == "pid.left_ki") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.left_ki must be >= 0");
                }
                updated.left_pid.ki = v;
            } else if (name == "pid.left_kd") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.left_kd must be >= 0");
                }
                updated.left_pid.kd = v;
            } else if (name == "pid.right_kp") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.right_kp must be >= 0");
                }
                updated.right_pid.kp = v;
            } else if (name == "pid.right_ki") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.right_ki must be >= 0");
                }
                updated.right_pid.ki = v;
            } else if (name == "pid.right_kd") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.right_kd must be >= 0");
                }
                updated.right_pid.kd = v;
            } else if (name == "pid.i_clamp") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.i_clamp must be >= 0");
                }
                updated.left_pid.i_clamp = v;
                updated.right_pid.i_clamp = v;
            } else if (name == "pid.output_limit_norm") {
                const double v = param.as_double();
                if (v <= 0.0 || v > 1.0) {
                    return fail("pid.output_limit_norm must be in (0, 1]");
                }
                updated.left_pid.output_limit_norm = v;
                updated.right_pid.output_limit_norm = v;
            } else if (name == "pid.stop_deadband_rpm") {
                const double v = param.as_double();
                if (v < 0.0) {
                    return fail("pid.stop_deadband_rpm must be >= 0");
                }
                updated.left_pid.stop_deadband_rpm = v;
                updated.right_pid.stop_deadband_rpm = v;
            } else if (name == "pid.velocity_alpha") {
                const double v = param.as_double();
                if (v < 0.0 || v > 1.0) {
                    return fail("pid.velocity_alpha must be in [0, 1]");
                }
                updated.velocity_alpha = v;
            } else if (name == "control.loop_ms") {
                const int v = param.as_int();
                if (v <= 0) {
                    return fail("control.loop_ms must be > 0");
                }
                updated.loop_ms = v;
            }
        } catch (const rclcpp::ParameterTypeException& ex) {
            return fail("Invalid type for '" + name + "': " + ex.what());
        }
    }

    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = updated;
    }

    return result;
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

        ControllerConfig config;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            config = config_;
        }

        const double linear = cmd_linear_.load();
        const double angular = cmd_angular_.load();
        const auto setpoints = payload::control_math::compute_wheel_setpoints(
            linear,
            angular,
            config.wheel_separation_m,
            config.wheel_radius_m,
            config.max_wheel_rpm);

        const int64_t left_count = left_encoder_ ? left_encoder_->count() : 0;
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

        prev_left_count_ = left_count;
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

        const float left_pwm_percent = static_cast<float>(
            std::clamp(std::abs(left_terms.output) * 100.0, 0.0, 100.0));
        const float right_pwm_percent = static_cast<float>(
            std::clamp(std::abs(right_terms.output) * 100.0, 0.0, 100.0));

        const bool left_forward = left_terms.output > 0.0;
        const bool left_reverse = left_terms.output < 0.0;
        const bool right_forward = right_terms.output > 0.0;
        const bool right_reverse = right_terms.output < 0.0;

        const int rc_l_in1 = lgGpioWrite(handle_, config.left_in1_pin, left_forward ? 1 : 0);
        const int rc_l_in2 = lgGpioWrite(handle_, config.left_in2_pin, left_reverse ? 1 : 0);
        const int rc_r_in1 = lgGpioWrite(handle_, config.right_in1_pin, right_forward ? 1 : 0);
        const int rc_r_in2 = lgGpioWrite(handle_, config.right_in2_pin, right_reverse ? 1 : 0);
        const int rc_l_pwm = lgTxPwm(handle_, config.left_pwm_pin, config.pwm_hz, left_pwm_percent, 0, 0);
        const int rc_r_pwm = lgTxPwm(handle_, config.right_pwm_pin, config.pwm_hz, right_pwm_percent, 0, 0);

        if (rc_l_in1 < 0 || rc_l_in2 < 0 ||
            rc_r_in1 < 0 || rc_r_in2 < 0 ||
            rc_l_pwm < 0 || rc_r_pwm < 0) {
            RCLCPP_WARN(node_->get_logger(),
                "GPIO | lgpio write error in=(%d,%d,%d,%d) pwm=(%d,%d)",
                rc_l_in1, rc_l_in2, rc_r_in1, rc_r_in2, rc_l_pwm, rc_r_pwm);
        }

        if (motor_state_pub_) {
            payload_interfaces::msg::MotorState msg;
            msg.linear_setpoint_mps = linear;
            msg.angular_setpoint_rad_s = angular;
            msg.left_setpoint_rpm = setpoints.left_rpm;
            msg.right_setpoint_rpm = setpoints.right_rpm;
            msg.left_measured_rpm = left_filtered_rpm_;
            msg.right_measured_rpm = right_filtered_rpm_;
            msg.left_pid_error_rpm = left_terms.error;
            msg.right_pid_error_rpm = right_terms.error;
            msg.left_pid_p = left_terms.p;
            msg.left_pid_i = left_terms.i;
            msg.left_pid_d = left_terms.d;
            msg.right_pid_p = right_terms.p;
            msg.right_pid_i = right_terms.i;
            msg.right_pid_d = right_terms.d;
            msg.left_output_norm = left_terms.output;
            msg.right_output_norm = right_terms.output;
            msg.left_pwm_percent = left_pwm_percent;
            msg.right_pwm_percent = right_pwm_percent;
            msg.left_encoder_count = left_count;
            msg.right_encoder_count = right_count;
            motor_state_pub_->publish(msg);
        }

        std::this_thread::sleep_until(
            loop_start + std::chrono::milliseconds(config.loop_ms));
    }
}

PLUGINLIB_EXPORT_CLASS(GPIOController, Controller)
