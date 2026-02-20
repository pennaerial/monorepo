#ifndef GPIO_CONTROLLER_HPP
#define GPIO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>

#include <atomic>
#include <thread>

#include "payload/controller.hpp"

// -------- Motor pins --------
constexpr int LEFT_IN1  = 5;   // PWM
constexpr int LEFT_IN2  = 4;
constexpr int RIGHT_IN1 = 6;   // PWM
constexpr int RIGHT_IN2 = 7;

// -------- Encoder pins --------
constexpr int ENC_LEFT_A  = 2;
constexpr int ENC_LEFT_B  = 10;
constexpr int ENC_RIGHT_A = 3;
constexpr int ENC_RIGHT_B = 11;

// -------- Control params --------
constexpr int   MAX_PWM     = 255;
constexpr float KP_STRAIGHT = 0.5f;
constexpr int   LOOP_MS     = 50;
constexpr int   PWM_HZ      = 200;


class GPIOController : public Controller {
public:
    GPIOController();
    ~GPIOController();

    void initialize(std::shared_ptr<rclcpp::Node> node) override;

    // Receives normalized linear and angular commands.
    // Converts to differential drive PWM with encoder straight correction.
    void drive_command(double linear, double angular) override;

    int handle_ {-1};  // lgpio chip handle (public for callback access)

private:
    // lgpio alert callbacks — fire on rising edge (LG_RISING_EDGE) of encoder A pins.
    static void enc_left_alert (int num_alerts, lgGpioAlert_p alerts, void* userdata);
    static void enc_right_alert(int num_alerts, lgGpioAlert_p alerts, void* userdata);

    // Runs every LOOP_MS ms: converts cmd_linear_/cmd_angular_ to
    // differential drive PWM and applies encoder straight correction.
    void control_loop();

    std::shared_ptr<rclcpp::Node> node_;

    std::atomic<long>   count_left_   {0};
    std::atomic<long>   count_right_  {0};
    std::atomic<double> cmd_linear_   {0.0};
    std::atomic<double> cmd_angular_  {0.0};
    std::atomic<bool>   running_      {false};

    std::thread control_thread_;
};

#endif
