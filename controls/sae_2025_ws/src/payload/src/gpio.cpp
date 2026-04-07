#include "payload/gpio.hpp"
#include <pigpio.h>

GPIO::GPIO(int pin, Direction direction) {
    pin_ = pin;
    direction_ = direction;
    int rc = 0;
    switch(direction_){
        case Direction::Input:
            rc = gpioSetMode(pin_, PI_INPUT);
            break;
        case Direction::Output:
            rc = gpioSetMode(pin_, PI_OUTPUT);
            if (rc >= 0) {
                gpioWrite(pin_, 0);
                gpioSetPWMrange(pin_, 10000);
            }
            break;
    }
    if (rc < 0) {
        RCLCPP_ERROR(logger(), "GPIO claim failed");
    }
}

void GPIO::write_high() {
    if (direction_ != Direction::Output) {
        RCLCPP_WARN(logger(), "Illegal Binary HIGH write on pin %d", pin_);
        return;
    }
    int rc = gpioWrite(pin_, 1);
    if (rc < 0) RCLCPP_WARN(logger(), "gpioWrite HIGH failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_low() {
    if (direction_ != Direction::Output) {
        RCLCPP_WARN(logger(), "Illegal Binary LOW write on pin %d", pin_);
        return;
    }
    int rc = gpioWrite(pin_, 0);
    if (rc < 0) RCLCPP_WARN(logger(), "gpioWrite LOW failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_pwm(float frequency, float duty_cycle) {
    if (direction_ != Direction::Output) {
        RCLCPP_WARN(logger(), "Illegal PWM write on pin %d", pin_);
        return;
    }
    int rc = gpioSetPWMfrequency(pin_, static_cast<unsigned>(frequency));
    if (rc < 0) {
        RCLCPP_WARN(logger(), "gpioSetPWMfrequency failed on pin %d: rc=%d", pin_, rc);
        return;
    }
    int duty_val = static_cast<int>(duty_cycle * 100.0f);
    rc = gpioPWM(pin_, duty_val);
    if (rc < 0) RCLCPP_WARN(logger(), "gpioPWM failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_servo(int pulse_width) {
    if (direction_ != Direction::Output) {
        RCLCPP_WARN(logger(), "Illegal servo write on pin %d", pin_);
        return;
    }
    int rc = gpioServo(pin_, pulse_width);
    if (rc < 0) RCLCPP_WARN(logger(), "gpioServo failed on pin %d: rc=%d", pin_, rc);
}

rclcpp::Logger GPIO::logger() {
    return rclcpp::get_logger("gpio");
}
