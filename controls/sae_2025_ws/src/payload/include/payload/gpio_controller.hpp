#ifndef GPIO_CONTROLLER_HPP
#define GPIO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "payload/controller.hpp"
#include "payload/control_math.hpp"
#include "payload/encoder.hpp"
#include "payload/motor.hpp"
#include "payload/payload_parameters.hpp"
#include "payload_interfaces/msg/motor_state.hpp"
#include "payload_interfaces/srv/compute_pid_ziegler_nichols.hpp"

class GPIOController : public Controller {
public:
    GPIOController();
    ~GPIOController() override;

    void initialize(std::shared_ptr<rclcpp::Node> node) override;
    void drive_command(double linear, double angular) override;

private:
    void compute_pid_zn_callback(
        const std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Request> request,
        std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Response> response);

    void control_loop();

    std::shared_ptr<rclcpp::Node> node_;
    int handle_ {-1};

    std::shared_ptr<payload::ParamListener> param_listener_;

    std::unique_ptr<SNMotor> left_motor_;
    std::unique_ptr<SNMotor> right_motor_;

    std::atomic<double> cmd_linear_  {0.0};
    std::atomic<double> cmd_angular_ {0.0};
    std::atomic<bool>   running_     {false};

    std::unique_ptr<QuadratureEncoder> left_encoder_;
    std::unique_ptr<QuadratureEncoder> right_encoder_;

    int64_t prev_left_count_  {0};
    int64_t prev_right_count_ {0};
    std::chrono::steady_clock::time_point prev_loop_time_ {};

    double left_filtered_rpm_  {0.0};
    double right_filtered_rpm_ {0.0};
    payload::control_math::PidState left_pid_state_  {};
    payload::control_math::PidState right_pid_state_ {};

    std::thread control_thread_;

    rclcpp::Publisher<payload_interfaces::msg::MotorState>::SharedPtr motor_state_pub_;
    rclcpp::Service<payload_interfaces::srv::ComputePidZieglerNichols>::SharedPtr zn_service_;
};

#endif
