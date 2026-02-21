#include "payload/motor.hpp"

Motor::Motor(int handle, int phase, int enable) {
    handle_ = handle;

    phase_ = std::make_unique<GPIO>(handle_, phase, static_cast<uint8_t>(PinType::Binary));
    enable_ = std::make_unique<GPIO>(handle_, enable, static_cast<uint8_t>(PinType::PWM));
}

void Motor::set_speed(float speed) {
    float norm_speed = std::clamp<float>(speed, -1.0f, 1.0f);
    if (norm_speed != speed) {
        RCLCPP_WARN(rclcpp::get_logger("motor"), "Clamped speed from %f to %f", speed, norm_speed);
    }

    if (norm_speed > 0) { //Forward, set phase to 0
        phase_->write_low();
    } else {
        phase_->write_high();
    }
    float pwm_duty_cycle = abs(norm_speed * 100.0f); //percentage
    enable_->write_pwm(500.0f, pwm_duty_cycle, 0, 0);
}