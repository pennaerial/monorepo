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
    RCLCPP_INFO(node_->get_logger(), "GPIO | Starting initialization...");

    // Try RPi 5 (gpiochip4) first, fall back to RPi 4 (gpiochip0)
    RCLCPP_INFO(node_->get_logger(), "GPIO | Trying gpiochip4 (RPi 5)...");
    handle_ = lgGpiochipOpen(4);
    if (handle_ < 0) {
        RCLCPP_INFO(node_->get_logger(), "GPIO | gpiochip4 failed (%d), trying gpiochip0 (RPi 4)...", handle_);
        handle_ = lgGpiochipOpen(0);
    }

    if (handle_ < 0) {
        RCLCPP_FATAL(node_->get_logger(),
            "GPIO | lgGpiochipOpen() failed (err=%d) — check /dev/gpiochip* permissions "
            "(try: sudo usermod -aG gpio $USER)", handle_);
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "GPIO | Opened gpiochip handle=%d", handle_);

    // Motor direction pins
    int rc;
    rc = lgGpioClaimOutput(handle_, 0, LEFT_IN2,  0);
    RCLCPP_INFO(node_->get_logger(), "GPIO | ClaimOutput LEFT_IN2  (BCM %d): rc=%d", LEFT_IN2,  rc);
    rc = lgGpioClaimOutput(handle_, 0, RIGHT_IN2, 0);
    RCLCPP_INFO(node_->get_logger(), "GPIO | ClaimOutput RIGHT_IN2 (BCM %d): rc=%d", RIGHT_IN2, rc);

    // Motor PWM pins
    rc = lgGpioClaimOutput(handle_, 0, LEFT_IN1,  0);
    RCLCPP_INFO(node_->get_logger(), "GPIO | ClaimOutput LEFT_IN1  (BCM %d): rc=%d", LEFT_IN1,  rc);
    rc = lgGpioClaimOutput(handle_, 0, RIGHT_IN1, 0);
    RCLCPP_INFO(node_->get_logger(), "GPIO | ClaimOutput RIGHT_IN1 (BCM %d): rc=%d", RIGHT_IN1, rc);

    rc = lgTxPwm(handle_, LEFT_IN1,  PWM_HZ, 0.0f, 0, 0);
    RCLCPP_INFO(node_->get_logger(), "GPIO | TxPwm LEFT_IN1  @ %d Hz: rc=%d", PWM_HZ, rc);
    rc = lgTxPwm(handle_, RIGHT_IN1, PWM_HZ, 0.0f, 0, 0);
    RCLCPP_INFO(node_->get_logger(), "GPIO | TxPwm RIGHT_IN1 @ %d Hz: rc=%d", PWM_HZ, rc);

    // Encoder B pins: input with pull-up, sampled inside alert callback
    rc = lgGpioClaimInput(handle_, LG_SET_BIAS_PULL_UP, ENC_LEFT_B);
    RCLCPP_INFO(node_->get_logger(), "GPIO | ClaimInput ENC_LEFT_B  (BCM %d): rc=%d", ENC_LEFT_B,  rc);
    rc = lgGpioClaimInput(handle_, LG_SET_BIAS_PULL_UP, ENC_RIGHT_B);
    RCLCPP_INFO(node_->get_logger(), "GPIO | ClaimInput ENC_RIGHT_B (BCM %d): rc=%d", ENC_RIGHT_B, rc);

    // Encoder A pins: alert on rising edge
    rc = lgGpioClaimAlert(handle_, LG_SET_BIAS_PULL_UP, LG_RISING_EDGE, ENC_LEFT_A,  -1);
    RCLCPP_INFO(node_->get_logger(), "GPIO | ClaimAlert ENC_LEFT_A  (BCM %d): rc=%d", ENC_LEFT_A,  rc);
    rc = lgGpioClaimAlert(handle_, LG_SET_BIAS_PULL_UP, LG_RISING_EDGE, ENC_RIGHT_A, -1);
    RCLCPP_INFO(node_->get_logger(), "GPIO | ClaimAlert ENC_RIGHT_A (BCM %d): rc=%d", ENC_RIGHT_A, rc);

    rc = lgGpioSetAlertsFunc(handle_, ENC_LEFT_A,  enc_left_alert,  this);
    RCLCPP_INFO(node_->get_logger(), "GPIO | SetAlertsFunc ENC_LEFT_A:  rc=%d", rc);
    rc = lgGpioSetAlertsFunc(handle_, ENC_RIGHT_A, enc_right_alert, this);
    RCLCPP_INFO(node_->get_logger(), "GPIO | SetAlertsFunc ENC_RIGHT_A: rc=%d", rc);

    running_ = true;
    control_thread_ = std::thread(&GPIOController::control_loop, this);

    RCLCPP_INFO(node_->get_logger(),
        "GPIO | Initialized — PWM @ %d Hz | "
        "Motors: L_PWM=BCM%d L_DIR=BCM%d R_PWM=BCM%d R_DIR=BCM%d | "
        "Encoders: LA=BCM%d LB=BCM%d RA=BCM%d RB=BCM%d",
        PWM_HZ,
        LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2,
        ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B);
}

void GPIOController::drive_command(double linear, double angular)
{
    RCLCPP_INFO(node_->get_logger(),
        "GPIO | drive_command received: linear=%.3f angular=%.3f", linear, angular);
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
    RCLCPP_INFO(node_->get_logger(), "GPIO | Control loop started (period=%d ms)", LOOP_MS);

    long last_left  = 0;
    long last_right = 0;
    int  loop_count = 0;

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

        int correction = 0;
        if (std::abs(angular) < 0.05) {
            correction = std::clamp(
                static_cast<int>(KP_STRAIGHT * (delta_left - delta_right)), -80, 80);
            base_left  -= correction;
            base_right += correction;
        }

        int duty_left  = std::clamp(base_left,  -MAX_PWM, MAX_PWM);
        int duty_right = std::clamp(base_right, -MAX_PWM, MAX_PWM);

        float pwm_left  = std::abs(duty_left)  * 100.0f / MAX_PWM;
        float pwm_right = std::abs(duty_right) * 100.0f / MAX_PWM;

        // Direction pin + PWM magnitude (lgTxPwm takes 0.0–100.0 duty cycle)
        int rc_l_dir = lgGpioWrite(handle_, LEFT_IN2,  duty_left  < 0 ? 1 : 0);
        int rc_r_dir = lgGpioWrite(handle_, RIGHT_IN2, duty_right < 0 ? 1 : 0);
        int rc_l_pwm = lgTxPwm(handle_, LEFT_IN1,  PWM_HZ, pwm_left,  0, 0);
        int rc_r_pwm = lgTxPwm(handle_, RIGHT_IN1, PWM_HZ, pwm_right, 0, 0);

        // Log every iteration at DEBUG, every 20 iterations (~1s) at INFO so it's visible
        // without --log-level debug
        if (loop_count % 20 == 0) {
            RCLCPP_INFO(node_->get_logger(),
                "GPIO | [%d] cmd=(lin=%.2f ang=%.2f) duty=(L:%d R:%d) "
                "pwm=(L:%.1f%% R:%.1f%%) dir=(L:%s R:%s) "
                "enc=(L:%ld R:%ld) corr=%d | rc=(%d %d %d %d)",
                loop_count,
                linear, angular,
                duty_left, duty_right,
                pwm_left, pwm_right,
                duty_left  < 0 ? "REV" : "FWD",
                duty_right < 0 ? "REV" : "FWD",
                cur_left, cur_right, correction,
                rc_l_dir, rc_r_dir, rc_l_pwm, rc_r_pwm);
        } else {
            RCLCPP_DEBUG(node_->get_logger(),
                "GPIO | cmd=(lin=%.2f ang=%.2f) duty=(L:%d R:%d) pwm=(L:%.1f%% R:%.1f%%)",
                linear, angular, duty_left, duty_right, pwm_left, pwm_right);
        }

        // Warn if any lgpio calls returned an error
        if (rc_l_dir < 0 || rc_r_dir < 0 || rc_l_pwm < 0 || rc_r_pwm < 0) {
            RCLCPP_WARN(node_->get_logger(),
                "GPIO | lgpio error in control loop: dir=(%d,%d) pwm=(%d,%d)",
                rc_l_dir, rc_r_dir, rc_l_pwm, rc_r_pwm);
        }

        loop_count++;
        std::this_thread::sleep_until(t0 + milliseconds(LOOP_MS));
    }

    RCLCPP_INFO(node_->get_logger(), "GPIO | Control loop exited after %d iterations", loop_count);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GPIOController, Controller)
