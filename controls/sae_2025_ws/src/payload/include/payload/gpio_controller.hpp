#ifndef GPIO_CONTROLLER_HPP
#define GPIO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>

#include <atomic>
#include <thread>

#include "payload/controller.hpp"

// -------- Motor pins (BCM numbering) --------
constexpr int LEFT_IN1  = 11;  // PWM  (physical 23) out A1
constexpr int LEFT_IN2  = 2;   //      (physical 3) out B1
constexpr int RIGHT_IN1 = 13;  // PWM  (physical 33) out A2
constexpr int RIGHT_IN2 = 3;   //      (physical 5) out B2

// -------- Encoder pins (BCM numbering) --------
constexpr int ENC_LEFT_A  = 19;  // interrupt (physical 35) Enc A1
constexpr int ENC_LEFT_B  = 23;  // sampled   (physical 16) Enc B1
constexpr int ENC_RIGHT_A = 21;  // interrupt (physical 40) Enc A2
constexpr int ENC_RIGHT_B = 27;  // sampled   (physical 13) Enc B2

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
