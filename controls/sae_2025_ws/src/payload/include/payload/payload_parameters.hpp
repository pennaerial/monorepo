#ifndef PAYLOAD_PARAMETERS_HPP
#define PAYLOAD_PARAMETERS_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace payload {

struct Pins {
  int LEFT_PWM = 18;
  int LEFT_IN1 = 15;
  int LEFT_IN2 = 14;
  int RIGHT_PWM = 13;
  int RIGHT_IN1 = 20;
  int RIGHT_IN2 = 16;
  int ENCA_CH1 = 6;
  int ENCA_CH2 = 5;
  int ENCB_CH1 = 10;
  int ENCB_CH2 = 9;
  int SERVO = 23;
};

struct Motor {
  double pwm_frequency = 200.0;
  double max_wheel_rpm = 270.0;
  double max_norm_speed = 1.0;
};

struct Encoder {
  int output_cpr = 617;
  int left_sign = 1;
  int right_sign = 1;
};

struct Kinematics {
  double wheel_radius_m = 0.01611839;
  double wheel_separation_m = 0.12132;
};

struct Pid {
  double left_kp = 0.005;
  double left_ki = 0.003;
  double left_kd = 0.0;
  double right_kp = 0.005;
  double right_ki = 0.003;
  double right_kd = 0.0;
  double i_clamp = 1.0;
  double output_limit_norm = 1.0;
  double stop_deadband_rpm = 1.0;
  double velocity_alpha = 1.0;
};

struct Control {
  int loop_ms = 50;
};

struct Servo {
  double pulse_min_us = 700.0;
  double pulse_max_us = 2300.0;
  int frequency = 50;
};

struct Sim {
  std::string world_name = "sae";
};

struct Params {
  std::string controller = "SimController";
  Pins pins;
  Motor motor;
  Encoder encoder;
  Kinematics kinematics;
  Pid pid;
  Control control;
  Servo servo;
  Sim sim;
};

class ParamListener {
 public:
  explicit ParamListener(rclcpp::Node* node) : node_(node) {
    const Params defaults;
    declare_if_missing("controller", defaults.controller);
    declare_if_missing("pins.LEFT_PWM", defaults.pins.LEFT_PWM);
    declare_if_missing("pins.LEFT_IN1", defaults.pins.LEFT_IN1);
    declare_if_missing("pins.LEFT_IN2", defaults.pins.LEFT_IN2);
    declare_if_missing("pins.RIGHT_PWM", defaults.pins.RIGHT_PWM);
    declare_if_missing("pins.RIGHT_IN1", defaults.pins.RIGHT_IN1);
    declare_if_missing("pins.RIGHT_IN2", defaults.pins.RIGHT_IN2);
    declare_if_missing("pins.ENCA_CH1", defaults.pins.ENCA_CH1);
    declare_if_missing("pins.ENCA_CH2", defaults.pins.ENCA_CH2);
    declare_if_missing("pins.ENCB_CH1", defaults.pins.ENCB_CH1);
    declare_if_missing("pins.ENCB_CH2", defaults.pins.ENCB_CH2);
    declare_if_missing("pins.SERVO", defaults.pins.SERVO);
    declare_if_missing("motor.pwm_frequency", defaults.motor.pwm_frequency);
    declare_if_missing("motor.max_wheel_rpm", defaults.motor.max_wheel_rpm);
    declare_if_missing("motor.max_norm_speed", defaults.motor.max_norm_speed);
    declare_if_missing("encoder.output_cpr", defaults.encoder.output_cpr);
    declare_if_missing("encoder.left_sign", defaults.encoder.left_sign);
    declare_if_missing("encoder.right_sign", defaults.encoder.right_sign);
    declare_if_missing(
        "kinematics.wheel_radius_m", defaults.kinematics.wheel_radius_m);
    declare_if_missing(
        "kinematics.wheel_separation_m",
        defaults.kinematics.wheel_separation_m);
    declare_if_missing("pid.left_kp", defaults.pid.left_kp);
    declare_if_missing("pid.left_ki", defaults.pid.left_ki);
    declare_if_missing("pid.left_kd", defaults.pid.left_kd);
    declare_if_missing("pid.right_kp", defaults.pid.right_kp);
    declare_if_missing("pid.right_ki", defaults.pid.right_ki);
    declare_if_missing("pid.right_kd", defaults.pid.right_kd);
    declare_if_missing("pid.i_clamp", defaults.pid.i_clamp);
    declare_if_missing(
        "pid.output_limit_norm", defaults.pid.output_limit_norm);
    declare_if_missing(
        "pid.stop_deadband_rpm", defaults.pid.stop_deadband_rpm);
    declare_if_missing("pid.velocity_alpha", defaults.pid.velocity_alpha);
    declare_if_missing("control.loop_ms", defaults.control.loop_ms);
    declare_if_missing("servo.pulse_min_us", defaults.servo.pulse_min_us);
    declare_if_missing("servo.pulse_max_us", defaults.servo.pulse_max_us);
    declare_if_missing("servo.frequency", defaults.servo.frequency);
    declare_if_missing("sim.world_name", defaults.sim.world_name);
  }

  explicit ParamListener(const std::shared_ptr<rclcpp::Node>& node)
      : ParamListener(node.get()) {}

  Params get_params() const {
    Params params{};
    params.controller = node_->get_parameter("controller").as_string();
    params.pins.LEFT_PWM = get<int>("pins.LEFT_PWM", params.pins.LEFT_PWM);
    params.pins.LEFT_IN1 = get<int>("pins.LEFT_IN1", params.pins.LEFT_IN1);
    params.pins.LEFT_IN2 = get<int>("pins.LEFT_IN2", params.pins.LEFT_IN2);
    params.pins.RIGHT_PWM = get<int>("pins.RIGHT_PWM", params.pins.RIGHT_PWM);
    params.pins.RIGHT_IN1 = get<int>("pins.RIGHT_IN1", params.pins.RIGHT_IN1);
    params.pins.RIGHT_IN2 = get<int>("pins.RIGHT_IN2", params.pins.RIGHT_IN2);
    params.pins.ENCA_CH1 = get<int>("pins.ENCA_CH1", params.pins.ENCA_CH1);
    params.pins.ENCA_CH2 = get<int>("pins.ENCA_CH2", params.pins.ENCA_CH2);
    params.pins.ENCB_CH1 = get<int>("pins.ENCB_CH1", params.pins.ENCB_CH1);
    params.pins.ENCB_CH2 = get<int>("pins.ENCB_CH2", params.pins.ENCB_CH2);
    params.pins.SERVO = get<int>("pins.SERVO", params.pins.SERVO);
    params.motor.pwm_frequency =
        get<double>("motor.pwm_frequency", params.motor.pwm_frequency);
    params.motor.max_wheel_rpm =
        get<double>("motor.max_wheel_rpm", params.motor.max_wheel_rpm);
    params.motor.max_norm_speed =
        get<double>("motor.max_norm_speed", params.motor.max_norm_speed);
    params.encoder.output_cpr =
        get<int>("encoder.output_cpr", params.encoder.output_cpr);
    params.encoder.left_sign =
        get<int>("encoder.left_sign", params.encoder.left_sign);
    params.encoder.right_sign =
        get<int>("encoder.right_sign", params.encoder.right_sign);
    params.kinematics.wheel_radius_m = get<double>(
        "kinematics.wheel_radius_m", params.kinematics.wheel_radius_m);
    params.kinematics.wheel_separation_m = get<double>(
        "kinematics.wheel_separation_m", params.kinematics.wheel_separation_m);
    params.pid.left_kp = get<double>("pid.left_kp", params.pid.left_kp);
    params.pid.left_ki = get<double>("pid.left_ki", params.pid.left_ki);
    params.pid.left_kd = get<double>("pid.left_kd", params.pid.left_kd);
    params.pid.right_kp = get<double>("pid.right_kp", params.pid.right_kp);
    params.pid.right_ki = get<double>("pid.right_ki", params.pid.right_ki);
    params.pid.right_kd = get<double>("pid.right_kd", params.pid.right_kd);
    params.pid.i_clamp = get<double>("pid.i_clamp", params.pid.i_clamp);
    params.pid.output_limit_norm = get<double>(
        "pid.output_limit_norm", params.pid.output_limit_norm);
    params.pid.stop_deadband_rpm = get<double>(
        "pid.stop_deadband_rpm", params.pid.stop_deadband_rpm);
    params.pid.velocity_alpha =
        get<double>("pid.velocity_alpha", params.pid.velocity_alpha);
    params.control.loop_ms =
        get<int>("control.loop_ms", params.control.loop_ms);
    params.servo.pulse_min_us =
        get<double>("servo.pulse_min_us", params.servo.pulse_min_us);
    params.servo.pulse_max_us =
        get<double>("servo.pulse_max_us", params.servo.pulse_max_us);
    params.servo.frequency =
        get<int>("servo.frequency", params.servo.frequency);
    params.sim.world_name =
        get<std::string>("sim.world_name", params.sim.world_name);
    return params;
  }

 private:
  template <typename T>
  void declare_if_missing(const std::string& name, const T& default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
  }

  template <typename T>
  T get(const std::string& name, const T& default_value) const {
    if (!node_->has_parameter(name)) {
      return default_value;
    }
    return node_->get_parameter(name).template get_value<T>();
  }

  rclcpp::Node* node_;
};

}  // namespace payload

#endif
