#include "payload/gpio_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono;


GPIOController::GPIOController() {}

GPIOController::~GPIOController()
{
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();

    if (handle_ >= 0) {
        lgTxPwm(handle_, LEFT_IN1,  0, 0.0f, 0, 0);
        lgTxPwm(handle_, RIGHT_IN1, 0, 0.0f, 0, 0);
        lgGpioWrite(handle_, LEFT_IN2,  0);
        lgGpioWrite(handle_, RIGHT_IN2, 0);
        lgGpiochipClose(handle_);
    }
}

void GPIOController::initialize(std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;

    // Try RPi 5 (gpiochip4) first, fall back to RPi 4 (gpiochip0)
    handle_ = lgGpiochipOpen(4);
    if (handle_ < 0) handle_ = lgGpiochipOpen(0);

    if (handle_ < 0) {
        RCLCPP_FATAL(node_->get_logger(),
            "GPIO | lgGpiochipOpen() failed — check /dev/gpiochip* permissions");
        return;
    }

    // Motor direction pins
    lgGpioClaimOutput(handle_, 0, LEFT_IN2,  0);
    lgGpioClaimOutput(handle_, 0, RIGHT_IN2, 0);

    // Motor PWM pins
    lgGpioClaimOutput(handle_, 0, LEFT_IN1,  0);
    lgGpioClaimOutput(handle_, 0, RIGHT_IN1, 0);
    lgTxPwm(handle_, LEFT_IN1,  PWM_HZ, 0.0f, 0, 0);
    lgTxPwm(handle_, RIGHT_IN1, PWM_HZ, 0.0f, 0, 0);

    // Encoder B pins: input with pull-up, sampled inside alert callback
    lgGpioClaimInput(handle_, LG_SET_BIAS_PULL_UP, ENC_LEFT_B);
    lgGpioClaimInput(handle_, LG_SET_BIAS_PULL_UP, ENC_RIGHT_B);

    // Encoder A pins: alert on rising edge
    lgGpioClaimAlert(handle_, LG_SET_BIAS_PULL_UP, LG_RISING_EDGE, ENC_LEFT_A,  -1);
    lgGpioClaimAlert(handle_, LG_SET_BIAS_PULL_UP, LG_RISING_EDGE, ENC_RIGHT_A, -1);
    lgGpioSetAlertsFunc(handle_, ENC_LEFT_A,  enc_left_alert,  this);
    lgGpioSetAlertsFunc(handle_, ENC_RIGHT_A, enc_right_alert, this);

    running_ = true;
    control_thread_ = std::thread(&GPIOController::control_loop, this);

    RCLCPP_INFO(node_->get_logger(), "GPIO | Initialized with lgpio (PWM @ %d Hz)", PWM_HZ);
}

void GPIOController::drive_command(double linear, double angular)
{
    cmd_linear_.store(linear);
    cmd_angular_.store(angular);
}

// -------- Encoder callbacks --------

void GPIOController::enc_left_alert(int num_alerts, lgGpioAlert_p alerts, void* userdata)
{
    auto* self = static_cast<GPIOController*>(userdata);
    for (int i = 0; i < num_alerts; i++) {
        if (alerts[i].report.level != 1) continue;
        if (lgGpioRead(self->handle_, ENC_LEFT_B) > 0) self->count_left_++;
        else                                            self->count_left_--;
    }
}

void GPIOController::enc_right_alert(int num_alerts, lgGpioAlert_p alerts, void* userdata)
{
    auto* self = static_cast<GPIOController*>(userdata);
    for (int i = 0; i < num_alerts; i++) {
        if (alerts[i].report.level != 1) continue;
        if (lgGpioRead(self->handle_, ENC_RIGHT_B) > 0) self->count_right_++;
        else                                             self->count_right_--;
    }
}

// -------- Control loop --------

void GPIOController::control_loop()
{
    long last_left  = 0;
    long last_right = 0;

    while (running_) {
        auto t0 = steady_clock::now();

        double linear  = cmd_linear_.load();
        double angular = cmd_angular_.load();

        // Differential drive: left = linear - angular, right = linear + angular
        int base_left  = static_cast<int>((linear - angular) * MAX_PWM);
        int base_right = static_cast<int>((linear + angular) * MAX_PWM);

        // Encoder straight correction — skip when intentionally turning
        long cur_left  = count_left_.load();
        long cur_right = count_right_.load();
        long delta_left  = cur_left  - last_left;
        long delta_right = cur_right - last_right;
        last_left  = cur_left;
        last_right = cur_right;

        if (std::abs(angular) < 0.05) {
            int correction = std::clamp(
                static_cast<int>(KP_STRAIGHT * (delta_left - delta_right)), -80, 80);
            base_left  -= correction;
            base_right += correction;
        }

        int duty_left  = std::clamp(base_left,  -MAX_PWM, MAX_PWM);
        int duty_right = std::clamp(base_right, -MAX_PWM, MAX_PWM);

        // Direction pin + PWM magnitude (lgTxPwm takes 0.0–100.0 duty cycle)
        lgGpioWrite(handle_, LEFT_IN2,  duty_left  < 0 ? 1 : 0);
        lgGpioWrite(handle_, RIGHT_IN2, duty_right < 0 ? 1 : 0);
        lgTxPwm(handle_, LEFT_IN1,  PWM_HZ, std::abs(duty_left)  * 100.0f / MAX_PWM, 0, 0);
        lgTxPwm(handle_, RIGHT_IN1, PWM_HZ, std::abs(duty_right) * 100.0f / MAX_PWM, 0, 0);

        RCLCPP_DEBUG(node_->get_logger(),
            "GPIO | dL:%ld dR:%ld | PWM L:%d R:%d",
            delta_left, delta_right, duty_left, duty_right);

        std::this_thread::sleep_until(t0 + milliseconds(LOOP_MS));
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GPIOController, Controller)
