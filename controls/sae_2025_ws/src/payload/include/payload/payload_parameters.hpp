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

struct ServoConfig {
  double pulse_min_us = 700.0;
  double pulse_max_us = 2300.0;
  int frequency = 50;
};

struct Sim {
  std::string world_name = "sae";
};

struct Params {
  std::string controller = "SimController";
  Pins pins {};
  Motor motor {};
  Encoder encoder {};
  Kinematics kinematics {};
  Pid pid {};
  Control control {};
  ServoConfig servo {};
  Sim sim {};
};

// Lightweight local replacement for generate_parameter_library.
class ParamListener {
 public:
  explicit ParamListener(rclcpp::Node* node) : node_(node) {
    declare_defaults();
  }

  explicit ParamListener(const std::shared_ptr<rclcpp::Node>& node)
      : ParamListener(node.get()) {}

  Params get_params() const {
    const Params defaults;
    Params params = defaults;

    params.controller = get_string("controller", defaults.controller);

    params.pins.LEFT_PWM = get_int("pins.LEFT_PWM", defaults.pins.LEFT_PWM);
    params.pins.LEFT_IN1 = get_int("pins.LEFT_IN1", defaults.pins.LEFT_IN1);
    params.pins.LEFT_IN2 = get_int("pins.LEFT_IN2", defaults.pins.LEFT_IN2);
    params.pins.RIGHT_PWM = get_int("pins.RIGHT_PWM", defaults.pins.RIGHT_PWM);
    params.pins.RIGHT_IN1 = get_int("pins.RIGHT_IN1", defaults.pins.RIGHT_IN1);
    params.pins.RIGHT_IN2 = get_int("pins.RIGHT_IN2", defaults.pins.RIGHT_IN2);
    params.pins.ENCA_CH1 = get_int("pins.ENCA_CH1", defaults.pins.ENCA_CH1);
    params.pins.ENCA_CH2 = get_int("pins.ENCA_CH2", defaults.pins.ENCA_CH2);
    params.pins.ENCB_CH1 = get_int("pins.ENCB_CH1", defaults.pins.ENCB_CH1);
    params.pins.ENCB_CH2 = get_int("pins.ENCB_CH2", defaults.pins.ENCB_CH2);
    params.pins.SERVO = get_int("pins.SERVO", defaults.pins.SERVO);

    params.motor.pwm_frequency =
        get_double("motor.pwm_frequency", defaults.motor.pwm_frequency);
    params.motor.max_wheel_rpm =
        get_double("motor.max_wheel_rpm", defaults.motor.max_wheel_rpm);
    params.motor.max_norm_speed =
        get_double("motor.max_norm_speed", defaults.motor.max_norm_speed);

    params.encoder.output_cpr =
        get_int("encoder.output_cpr", defaults.encoder.output_cpr);
    params.encoder.left_sign =
        get_int("encoder.left_sign", defaults.encoder.left_sign);
    params.encoder.right_sign =
        get_int("encoder.right_sign", defaults.encoder.right_sign);

    params.kinematics.wheel_radius_m =
        get_double("kinematics.wheel_radius_m", defaults.kinematics.wheel_radius_m);
    params.kinematics.wheel_separation_m = get_double(
        "kinematics.wheel_separation_m", defaults.kinematics.wheel_separation_m);

    params.pid.left_kp = get_double("pid.left_kp", defaults.pid.left_kp);
    params.pid.left_ki = get_double("pid.left_ki", defaults.pid.left_ki);
    params.pid.left_kd = get_double("pid.left_kd", defaults.pid.left_kd);
    params.pid.right_kp = get_double("pid.right_kp", defaults.pid.right_kp);
    params.pid.right_ki = get_double("pid.right_ki", defaults.pid.right_ki);
    params.pid.right_kd = get_double("pid.right_kd", defaults.pid.right_kd);
    params.pid.i_clamp = get_double("pid.i_clamp", defaults.pid.i_clamp);
    params.pid.output_limit_norm =
        get_double("pid.output_limit_norm", defaults.pid.output_limit_norm);
    params.pid.stop_deadband_rpm =
        get_double("pid.stop_deadband_rpm", defaults.pid.stop_deadband_rpm);
    params.pid.velocity_alpha =
        get_double("pid.velocity_alpha", defaults.pid.velocity_alpha);

    params.control.loop_ms = get_int("control.loop_ms", defaults.control.loop_ms);

    params.servo.pulse_min_us =
        get_double("servo.pulse_min_us", defaults.servo.pulse_min_us);
    params.servo.pulse_max_us =
        get_double("servo.pulse_max_us", defaults.servo.pulse_max_us);
    params.servo.frequency = get_int("servo.frequency", defaults.servo.frequency);

    params.sim.world_name =
        get_string("sim.world_name", defaults.sim.world_name);

    return params;
  }

 private:
  void declare_defaults() {
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
        "kinematics.wheel_separation_m", defaults.kinematics.wheel_separation_m);

    declare_if_missing("pid.left_kp", defaults.pid.left_kp);
    declare_if_missing("pid.left_ki", defaults.pid.left_ki);
    declare_if_missing("pid.left_kd", defaults.pid.left_kd);
    declare_if_missing("pid.right_kp", defaults.pid.right_kp);
    declare_if_missing("pid.right_ki", defaults.pid.right_ki);
    declare_if_missing("pid.right_kd", defaults.pid.right_kd);
    declare_if_missing("pid.i_clamp", defaults.pid.i_clamp);
    declare_if_missing("pid.output_limit_norm", defaults.pid.output_limit_norm);
    declare_if_missing("pid.stop_deadband_rpm", defaults.pid.stop_deadband_rpm);
    declare_if_missing("pid.velocity_alpha", defaults.pid.velocity_alpha);

    declare_if_missing("control.loop_ms", defaults.control.loop_ms);

    declare_if_missing("servo.pulse_min_us", defaults.servo.pulse_min_us);
    declare_if_missing("servo.pulse_max_us", defaults.servo.pulse_max_us);
    declare_if_missing("servo.frequency", defaults.servo.frequency);

    declare_if_missing("sim.world_name", defaults.sim.world_name);
  }

  template <typename T>
  void declare_if_missing(const std::string& name, const T& default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
  }

  int get_int(const std::string& name, int default_value) const {
    (void)default_value;
    return static_cast<int>(node_->get_parameter(name).as_int());
  }

  double get_double(const std::string& name, double default_value) const {
    (void)default_value;
    return node_->get_parameter(name).as_double();
  }

  std::string get_string(
      const std::string& name, const std::string& default_value) const {
    (void)default_value;
    return node_->get_parameter(name).as_string();
  }

  rclcpp::Node* node_;
};

}  // namespace payload

#endif
