#include "payload/gpio_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <pigpiod_if2.h>
#include <pluginlib/class_list_macros.hpp>

GPIOController::GPIOController() {}

GPIOController::~GPIOController()
{
  if (node_ != nullptr) {
    RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN GPIO CONTROLLER");
  }

  safe_shutdown();

  left_encoder_.reset();
  right_encoder_.reset();
  left_motor_.reset();
  right_motor_.reset();
  servo_.reset();

  motor_state_pub_.reset();
  zn_service_.reset();
  dead_reckon_srv_.reset();
  dead_reckon_cbg_.reset();
  param_listener_.reset();
  if (initialized_ && pi_ >= 0) {
    pigpio_stop(pi_);
    pi_ = -1;
    initialized_ = false;
  }
}

void GPIOController::initialize(rclcpp::Node * node)
{
  node_ = node;
  RCLCPP_INFO(node_->get_logger(), "GPIO | Starting initialization...");

  param_listener_ = std::make_shared<payload::ParamListener>(node_);
  const auto p = param_listener_->get_params();

  pi_ = pigpio_start(nullptr, nullptr);
  if (pi_ < 0) {
    RCLCPP_FATAL(
      node_->get_logger(),
      "GPIO | pigpio_start() failed (err=%d). Is pigpiod running? Try: sudo systemctl start pigpiod",
      pi_);
    throw std::runtime_error(
            "GPIOController failed to connect to pigpiod daemon.");
  }
  initialized_ = true;

  const int pwm_hz = std::max(
    1, static_cast<int>(std::lround(p.motor.pwm_frequency)));

  left_motor_ = std::make_unique<SNMotor>(
    pi_,
    p.pins.LEFT_PWM,
    p.pins.LEFT_IN1,
    p.pins.LEFT_IN2,
    pwm_hz,
    MotorType::LEFT);
  right_motor_ = std::make_unique<SNMotor>(
    pi_,
    p.pins.RIGHT_PWM,
    p.pins.RIGHT_IN1,
    p.pins.RIGHT_IN2,
    pwm_hz,
    MotorType::RIGHT);

  left_encoder_ = std::make_unique<QuadratureEncoder>(
    pi_,
    p.pins.ENCB_CH1,
    p.pins.ENCB_CH2,
    p.encoder.output_cpr,
    MotorType::LEFT);
  right_encoder_ = std::make_unique<QuadratureEncoder>(
    pi_,
    p.pins.ENCA_CH1,
    p.pins.ENCA_CH2,
    p.encoder.output_cpr,
    MotorType::RIGHT);

  servo_ = std::make_unique<Servo>(
    pi_,
    p.pins.SERVO,
    p.servo.frequency,
    static_cast<float>(p.servo.pulse_min_us),
    static_cast<float>(p.servo.pulse_max_us));

  prev_left_count_ = left_encoder_->count();
  prev_right_count_ = right_encoder_->count();
  prev_loop_time_ = std::chrono::steady_clock::now();

  left_filtered_rpm_ = 0.0;
  right_filtered_rpm_ = 0.0;
  left_pid_state_ = payload::control_math::PidState{};
  right_pid_state_ = payload::control_math::PidState{};

  const std::string state_topic = "motor_state";
  const std::string zn_service_name = "compute_pid_zn";

  motor_state_pub_ =
    node_->create_publisher<payload_interfaces::msg::MotorState>(state_topic, 10);
  zn_service_ =
    node_->create_service<payload_interfaces::srv::ComputePidZieglerNichols>(
    zn_service_name,
    [this](
      const std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Request>
      request,
      std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Response>
      response) {compute_pid_zn_callback(request, response);});

  dead_reckon_cbg_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  const std::string dr_service_name = "dead_reckon";
  dead_reckon_srv_ = node_->create_service<payload_interfaces::srv::DeadReckon>(
    dr_service_name,
    std::bind(
      &GPIOController::dead_reckon_callback, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    dead_reckon_cbg_);

  running_ = true;
  control_thread_ = std::thread(&GPIOController::control_loop, this);

  RCLCPP_INFO(
    node_->get_logger(),
    "GPIO | Ready. state_topic=%s zn_service=%s dr_service=%s",
    state_topic.c_str(), zn_service_name.c_str(), dr_service_name.c_str());
}

void GPIOController::drive_command(double linear, double angular)
{
  if (shutdown_requested_.load()) {
    return;
  }
  if (control_mode_.load() == ControlMode::DEAD_RECKONING) {
    return;
  }
  cmd_linear_.store(linear);
  cmd_angular_.store(angular);
}

void GPIOController::servo_command(double degree)
{
  if (shutdown_requested_.load()) {
    return;
  }
  if (servo_) {
    servo_->degree_setpoint(static_cast<float>(degree));
  }
}

void GPIOController::safe_shutdown()
{
  if (shutdown_requested_.exchange(true)) {
    return;
  }

  cmd_linear_.store(0.0);
  cmd_angular_.store(0.0);
  dr_left_done_.store(true);
  dr_right_done_.store(true);
  control_mode_.store(ControlMode::NORMAL);
  running_ = false;

  if (control_thread_.joinable()) {
    if (node_ != nullptr) {
      RCLCPP_INFO(node_->get_logger(), "Joining control loop");
    }
    control_thread_.join();
  }

  if (left_motor_) {
    left_motor_->coast();
  }

  if (right_motor_) {
    right_motor_->coast();
  }

  if (servo_) {
    servo_->degree_setpoint(0.0f);
  }
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

void GPIOController::dead_reckon_callback(
  const std::shared_ptr<payload_interfaces::srv::DeadReckon::Request> request,
  std::shared_ptr<payload_interfaces::srv::DeadReckon::Response> response)
{
  if (shutdown_requested_.load()) {
    response->success = false;
    response->message = "Controller is shutting down";
    return;
  }

  const bool has_linear = std::abs(request->linear) > 1e-9;
  const bool has_angular = std::abs(request->angular) > 1e-9;

  if (has_linear && has_angular) {
    response->success = false;
    response->message = "Provide only linear OR angular, not both";
    return;
  }
  if (!has_linear && !has_angular) {
    response->success = false;
    response->message = "Both linear and angular are zero";
    return;
  }
  if (request->speed <= 0.0) {
    response->success = false;
    response->message = "speed must be > 0";
    return;
  }

  const auto p = param_listener_->get_params();
  const double cpr = static_cast<double>(p.encoder.output_cpr);
  const double r = p.kinematics.wheel_radius_m;
  const double L = p.kinematics.wheel_separation_m;

  int64_t target_counts;
  double linear_cmd = 0.0;
  double angular_cmd = 0.0;

  if (has_linear) {
    target_counts = static_cast<int64_t>(
      std::ceil(std::abs(request->linear) * cpr / (2.0 * M_PI * r)));
    linear_cmd = std::copysign(request->speed, request->linear);
  } else {
    target_counts = static_cast<int64_t>(
      std::ceil((L / 2.0) * std::abs(request->angular) * cpr / (2.0 * M_PI * r)));
    angular_cmd = std::copysign(request->speed, request->angular);
  }

  const int64_t left_now = left_encoder_ ? left_encoder_->count() : 0;
  const int64_t right_now = right_encoder_ ? right_encoder_->count() : 0;

  // Compute per-wheel direction in raw encoder space.
  // left_sign/right_sign normalise so forward = positive normalised count.
  // For angular motion the wheels spin opposite each other.
  int64_t left_dir;
  int64_t right_dir;
  if (has_linear) {
    const int64_t motion_dir = (request->linear > 0) ? 1 : -1;
    left_dir = motion_dir * static_cast<int64_t>(p.encoder.left_sign);
    right_dir = motion_dir * static_cast<int64_t>(p.encoder.right_sign);
  } else {
    // angular > 0 → CCW: left wheel backward, right wheel forward (normalised)
    const int64_t angular_dir = (request->angular > 0) ? 1 : -1;
    left_dir = -angular_dir * static_cast<int64_t>(p.encoder.left_sign);
    right_dir = angular_dir * static_cast<int64_t>(p.encoder.right_sign);
  }

  const int64_t left_goal = left_now + target_counts * left_dir;
  const int64_t right_goal = right_now + target_counts * right_dir;

  RCLCPP_INFO(
    node_->get_logger(),
    "DeadReckon | target=%ld counts  left_start=%ld goal=%ld  right_start=%ld goal=%ld  linear=%.3f  angular=%.3f",
    target_counts, left_now, left_goal, right_now, right_goal, linear_cmd, angular_cmd);

  dr_left_start_.store(left_now);
  dr_right_start_.store(right_now);
  dr_left_goal_.store(left_goal);
  dr_right_goal_.store(right_goal);
  dr_left_done_.store(false);
  dr_right_done_.store(false);

  cmd_linear_.store(linear_cmd);
  cmd_angular_.store(angular_cmd);
  control_mode_.store(ControlMode::DEAD_RECKONING);

  while (control_mode_.load() == ControlMode::DEAD_RECKONING && !shutdown_requested_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (shutdown_requested_.load()) {
    response->success = false;
    response->message = "Controller is shutting down";
    return;
  }

  response->success = true;
  response->message = "Dead reckoning complete";
}

void GPIOController::control_loop()
{
  while (running_) {
    const auto loop_start = std::chrono::steady_clock::now();
    const auto p = param_listener_->get_params();

    const double linear = cmd_linear_.load();
    const double angular = cmd_angular_.load();
    const auto setpoints = payload::control_math::compute_wheel_setpoints(
      linear,
      angular,
      p.kinematics.wheel_separation_m,
      p.kinematics.wheel_radius_m,
      p.motor.max_wheel_rpm);

    const int64_t left_count = left_encoder_->count();
    const int64_t right_count = right_encoder_->count();

    const auto now = std::chrono::steady_clock::now();
    double dt_s = std::chrono::duration<double>(now - prev_loop_time_).count();
    if (dt_s <= 0.0) {
      dt_s = static_cast<double>(p.control.loop_ms) / 1000.0;
    }
    prev_loop_time_ = now;

    const int64_t left_delta =
      (left_count - prev_left_count_) * static_cast<int64_t>(p.encoder.left_sign);
    const int64_t right_delta =
      (right_count - prev_right_count_) * static_cast<int64_t>(p.encoder.right_sign);

    prev_left_count_ = left_count;
    prev_right_count_ = right_count;

    // Declare pub fields with defaults so they're valid in all modes
    payload::control_math::PidTerms left_terms{};
    payload::control_math::PidTerms right_terms{};
    float left_duty = 0.0f;
    float right_duty = 0.0f;

    if (control_mode_.load() == ControlMode::DEAD_RECKONING) {
      const int64_t left_goal = dr_left_goal_.load();
      const int64_t right_goal = dr_right_goal_.load();
      const int64_t left_start = dr_left_start_.load();
      const int64_t right_start = dr_right_start_.load();

      // Done when count has reached or passed goal in the intended direction.
      // Direction is inferred from goal vs start — no separate sign param needed.
      const bool left_done = (left_goal >= left_start) ?
        (left_count >= left_goal) :
        (left_count <= left_goal);
      const bool right_done = (right_goal >= right_start) ?
        (right_count >= right_goal) :
        (right_count <= right_goal);

      const int64_t left_rem = std::abs(left_goal - left_count);
      const int64_t right_rem = std::abs(right_goal - right_count);

      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 200,
        "DR | left=%ld goal=%ld rem=%ld done=%d  right=%ld goal=%ld rem=%ld done=%d",
        left_count, left_goal, left_rem, left_done,
        right_count, right_goal, right_rem, right_done);

      if (left_done && !dr_left_done_.load()) {
        dr_left_done_.store(true);
        RCLCPP_INFO(node_->get_logger(), "DR | Left wheel done (rem=%ld)", left_rem);
      }
      if (right_done && !dr_right_done_.load()) {
        dr_right_done_.store(true);
        RCLCPP_INFO(node_->get_logger(), "DR | Right wheel done (rem=%ld)", right_rem);
      }

      if (dr_left_done_.load() && dr_right_done_.load()) {
        cmd_linear_.store(0.0);
        cmd_angular_.store(0.0);
        if (left_motor_) {left_motor_->hard_brake();}
        if (right_motor_) {right_motor_->hard_brake();}
        control_mode_.store(ControlMode::NORMAL);
        RCLCPP_INFO(node_->get_logger(), "DR | Complete — braking motors");
      }

    }

    {
      const double left_raw_rpm = payload::control_math::rpm_from_count_delta(
        left_delta, p.encoder.output_cpr, dt_s);
      const double right_raw_rpm = payload::control_math::rpm_from_count_delta(
        right_delta, p.encoder.output_cpr, dt_s);

      left_filtered_rpm_ = payload::control_math::low_pass_filter(
        left_raw_rpm, left_filtered_rpm_, p.pid.velocity_alpha);
      right_filtered_rpm_ = payload::control_math::low_pass_filter(
        right_raw_rpm, right_filtered_rpm_, p.pid.velocity_alpha);

      const payload::control_math::PidConfig left_pid_cfg {
        p.pid.left_kp,
        p.pid.left_ki,
        p.pid.left_kd,
        p.pid.i_clamp,
        p.pid.output_limit_norm,
        p.pid.stop_deadband_rpm,
      };
      const payload::control_math::PidConfig right_pid_cfg {
        p.pid.right_kp,
        p.pid.right_ki,
        p.pid.right_kd,
        p.pid.i_clamp,
        p.pid.output_limit_norm,
        p.pid.stop_deadband_rpm,
      };

      const bool in_dr = (control_mode_.load() == ControlMode::DEAD_RECKONING);
      const double left_sp = (in_dr && dr_left_done_.load()) ? 0.0 : setpoints.left_rpm;
      const double right_sp = (in_dr && dr_right_done_.load()) ? 0.0 : setpoints.right_rpm;

      left_terms = payload::control_math::pid_step(
        left_sp,
        left_filtered_rpm_,
        dt_s,
        left_pid_cfg,
        left_pid_state_);
      right_terms = payload::control_math::pid_step(
        right_sp,
        right_filtered_rpm_,
        dt_s,
        right_pid_cfg,
        right_pid_state_);

      left_duty = static_cast<float>(std::abs(left_terms.output) * 100.0);
      right_duty = static_cast<float>(std::abs(right_terms.output) * 100.0);

      if (left_terms.output > 0.0) {
        left_motor_->forward(left_duty);
      } else if (left_terms.output < 0.0) {
        left_motor_->reverse(left_duty);
      } else {
        left_motor_->coast();
      }

      if (right_terms.output > 0.0) {
        right_motor_->forward(right_duty);
      } else if (right_terms.output < 0.0) {
        right_motor_->reverse(right_duty);
      } else {
        right_motor_->coast();
      }

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
      msg.left_pwm_percent = std::clamp(left_duty, 0.0f, 100.0f);
      msg.right_pwm_percent = std::clamp(right_duty, 0.0f, 100.0f);
      msg.left_encoder_count = left_count;
      msg.right_encoder_count = right_count;
      motor_state_pub_->publish(msg);
    }

    std::this_thread::sleep_until(
      loop_start + std::chrono::milliseconds(p.control.loop_ms));
  }
}

PLUGINLIB_EXPORT_CLASS(GPIOController, Controller)
