#ifndef GPIO_CONTROLLER_HPP
#define GPIO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <lgpio.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "payload/controller.hpp"
#include "payload/control_math.hpp"
#include "payload/encoder.hpp"
#include "payload_interfaces/msg/motor_state.hpp"
#include "payload_interfaces/srv/compute_pid_ziegler_nichols.hpp"

class GPIOController : public Controller {
public:
    GPIOController();
    ~GPIOController() override;

    void initialize(rclcpp::Node* node) override;
    void drive_command(double linear, double angular) override;

private:
    struct ControllerConfig {
        int left_pwm_pin {13};
        int left_in1_pin {16};
        int left_in2_pin {20};
        int right_pwm_pin {18};
        int right_in1_pin {15};
        int right_in2_pin {14};

        int enc_left_a {0};
        int enc_left_b {9};
        int enc_right_a {11};
        int enc_right_b {10};

        int pwm_hz {500};
        int loop_ms {50};

        int encoder_output_cpr {617};
        int encoder_left_sign {1};
        int encoder_right_sign {1};

        double wheel_radius_m {0.01611839};
        double wheel_separation_m {0.12132};
        double max_wheel_rpm {120.0};

        payload::control_math::PidConfig left_pid {
            0.02,
            0.0,
            0.0,
            1.0,
            1.0,
            1.0,
        };
        payload::control_math::PidConfig right_pid {
            0.02,
            0.0,
            0.0,
            1.0,
            1.0,
            1.0,
        };
        double velocity_alpha {1.0};
    };

    void load_initial_config_from_parameters();
    rcl_interfaces::msg::SetParametersResult on_parameters_set(
        const std::vector<rclcpp::Parameter>& params);

    void compute_pid_zn_callback(
        const std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Request> request,
        std::shared_ptr<payload_interfaces::srv::ComputePidZieglerNichols::Response> response);

    void control_loop();

    int handle_ {-1};

    std::mutex config_mutex_;
    ControllerConfig config_;

    std::atomic<double> cmd_linear_ {0.0};
    std::atomic<double> cmd_angular_ {0.0};
    std::atomic<bool> running_ {false};

    std::unique_ptr<QuadratureEncoder> left_encoder_;
    std::unique_ptr<QuadratureEncoder> right_encoder_;

    int64_t prev_left_count_ {0};
    int64_t prev_right_count_ {0};
    std::chrono::steady_clock::time_point prev_loop_time_ {};

    double left_filtered_rpm_ {0.0};
    double right_filtered_rpm_ {0.0};
    payload::control_math::PidState left_pid_state_ {};
    payload::control_math::PidState right_pid_state_ {};

    std::thread control_thread_;

    rclcpp::Publisher<payload_interfaces::msg::MotorState>::SharedPtr motor_state_pub_;
    rclcpp::Service<payload_interfaces::srv::ComputePidZieglerNichols>::SharedPtr zn_service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif
