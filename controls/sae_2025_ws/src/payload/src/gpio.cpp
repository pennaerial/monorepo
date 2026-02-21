#include "payload/gpio.hpp"
#include <lgpio.h>

GPIO::GPIO(int handle, int pin, uint8_t permissions) {
    handle_ = handle;
    pin_ = pin;
    permissions_ = permissions;
}

void GPIO::write_high() {
    if (!allowed(PinType::Binary)) {
        RCLCPP_WARN(logger(), "Illegal Binary HIGH write on pin %d", pin_);
        return;
    }
    int rc = lgGpioWrite(handle_, pin_, 1);
    if (rc < 0) RCLCPP_WARN(logger(), "lgGpioWrite HIGH failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_low() {
    if (!allowed(PinType::Binary)) {
        RCLCPP_WARN(logger(), "Illegal Binary LOW write on pin %d", pin_);
        return;
    }
    int rc = lgGpioWrite(handle_, pin_, 0);
    if (rc < 0) RCLCPP_WARN(logger(), "lgGpioWrite LOW failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_pwm(float frequency, float duty_cycle, int offset, int cycles) {
    if (!allowed(PinType::PWM)) {
        RCLCPP_WARN(logger(), "Illegal PWM write on pin %d", pin_);
        return;
    }
    //for continuous and immediate, set offset and cycles to 0
    int rc = lgTxPwm(handle_, pin_, frequency, duty_cycle, offset, cycles);
    if (rc < 0) RCLCPP_WARN(logger(), "lgTxPwm failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_servo(int pulse_width, int frequency, int offset, int cycles) {
    if (!allowed(PinType::Servo)) {
        RCLCPP_WARN(logger(), "Illegal servo write on pin %d", pin_);
        return;
    }
    int rc = lgTxServo(handle_, pin_, pulse_width, frequency, offset, cycles);
    if (rc < 0) RCLCPP_WARN(logger(), "lgTxServo failed on pin %d: rc=%d", pin_, rc);
}

rclcpp::Logger GPIO::logger() {
    return rclcpp::get_logger("gpio");
}

//Uses bitwise and operator and conversion from uint8_t -> bool
bool GPIO::allowed(PinType pin_type) {
    return permissions_ & static_cast<uint8_t>(pin_type);
}