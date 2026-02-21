#include "payload/motor.hpp"

Motor::Motor(int handle, int phase, int enable) {
    handle_ = handle;

    uint8_t bin_pwm_perm = static_cast<uint8_t>(PinType::Binary) | static_cast<uint8_t>(PinType::PWM);
    phase_ = std::make_unique<GPIO>(handle_, phase, bin_pwm_perm); //in1
    enable_ = std::make_unique<GPIO>(handle_, enable, bin_pwm_perm); //in2
}

void Motor::set_speed(float speed) {
    float norm_speed = std::clamp<float>(speed, -1.0f, 1.0f);
    if (norm_speed != speed) {
        RCLCPP_WARN(rclcpp::get_logger("motor"), "Clamped speed from %f to %f", speed, norm_speed);
    }

    float pwm_duty_cycle = abs(norm_speed * 100.0f); //percentage
    if (norm_speed > 0) { //forward, set phase to 0
        phase_->write_high();
        enable_->write_pwm(500.0f, pwm_duty_cycle, 0, 0);
    } else if (norm_speed < 0){ //reverse
        phase_->write_pwm(500.0f, pwm_duty_cycle, 0, 0);
        enable_->write_high();
    } else { //brake
        phase_->write_high();
        enable_->write_high();
    }



}