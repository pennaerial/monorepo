#ifndef GPIO_CONTROLLER_HPP
#define GPIO_CONTROLLER_HPP

#include <pigpiod_if2.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "payload_controller/control_math.hpp"
#include "payload_controller/controller.hpp"
#include "payload_controller/encoder.hpp"
#include "payload_controller/motor.hpp"
#include "payload_controller/payload_controller_parameters.hpp"
#include "payload_controller/servo.hpp"
#include "payload_interfaces/msg/motor_state.hpp"
#include "payload_interfaces/srv/compute_pid_ziegler_nichols.hpp"
#include "payload_interfaces/srv/dead_reckon.hpp"

enum class ControlMode {
  NORMAL,
  DEAD_RECKONING
};

class GPIOController : public Controller
{
public:
  GPIOController();
  GPIOController(std::unique_ptr<Motor> left_motor, std::unique_ptr<Motor> right_motor);
  ~GPIOController() override;

  void initialize(rclcpp::Node* node) override;
  void drive_command(double linear, double angular) override;
  void servo_command(double degree) override;
  void safe_shutdown() override;

private:
  void compute_pid_zn_callback(
      const std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Request> request,
      std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Response> response
  );

  void dead_reckon_callback(
      const std::shared_ptr<payload_interfaces::srv::DeadReckon::Request> request,
      std::shared_ptr<payload_interfaces::srv::DeadReckon::Response> response
  );

  void control_loop();

  rclcpp::Node* node_{nullptr};
  bool initialized_{false};
  int pi_{-1};

  std::shared_ptr<payload::ParamListener> param_listener_;

  std::unique_ptr<Motor> left_motor_;
  std::unique_ptr<Motor> right_motor_;

  std::atomic<double> cmd_linear_{0.0};
  std::atomic<double> cmd_angular_{0.0};
  std::atomic<bool> running_{false};
  std::atomic<bool> shutdown_requested_{false};

  std::unique_ptr<QuadratureEncoder> left_encoder_;
  std::unique_ptr<QuadratureEncoder> right_encoder_;

  int64_t prev_left_count_{0};
  int64_t prev_right_count_{0};
  std::chrono::steady_clock::time_point prev_loop_time_{};

  double left_filtered_rpm_{0.0};
  double right_filtered_rpm_{0.0};
  payload::control_math::PidState left_pid_state_{};
  payload::control_math::PidState right_pid_state_{};

  std::unique_ptr<Servo> servo_;

  std::thread control_thread_;

  // Dead reckoning state
  std::atomic<ControlMode> control_mode_{ControlMode::NORMAL};
  std::atomic<int64_t> dr_left_goal_{0};  // absolute encoder count to reach
  std::atomic<int64_t> dr_right_goal_{0};
  std::atomic<int64_t> dr_left_start_{0};  // snapshot at service call time
  std::atomic<int64_t> dr_right_start_{0};
  std::atomic<bool> dr_left_done_{false};
  std::atomic<bool> dr_right_done_{false};

  rclcpp::Publisher<payload_interfaces::msg::MotorState>::SharedPtr motor_state_pub_;
  rclcpp::Service<payload_interfaces::srv::ComputePidZieglerNichols>::SharedPtr zn_service_;
  rclcpp::CallbackGroup::SharedPtr dead_reckon_cbg_;
  rclcpp::Service<payload_interfaces::srv::DeadReckon>::SharedPtr dead_reckon_srv_;
};

#endif
