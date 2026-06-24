#include "payload_controller/gpio.hpp"
#include <pigpiod_if2.h>

GPIO::GPIO(int pi, int pin, Direction direction)
: pi_(pi), pin_(pin), direction_(direction)
{
  int rc = 0;
  switch (direction_) {
    case Direction::Input:
      rc = set_mode(pi_, pin_, PI_INPUT);
      break;
    case Direction::Output:
      rc = set_mode(pi_, pin_, PI_OUTPUT);
      if (rc == 0) {
        gpio_write(pi_, pin_, 0);
        set_PWM_range(pi_, pin_, 10000);
      }
      break;
  }
  if (rc != 0) {
    RCLCPP_ERROR(logger(), "GPIO claim failed on pin %d (rc=%d)", pin_, rc);
  }
}

void GPIO::write_high()
{
  if (direction_ != Direction::Output) {
    RCLCPP_WARN(logger(), "Illegal Binary HIGH write on pin %d", pin_);
    return;
  }
  int rc = gpio_write(pi_, pin_, 1);
  if (rc != 0) {RCLCPP_WARN(logger(), "gpio_write HIGH failed on pin %d: rc=%d", pin_, rc);}
}

void GPIO::write_low()
{
  if (direction_ != Direction::Output) {
    RCLCPP_WARN(logger(), "Illegal Binary LOW write on pin %d", pin_);
    return;
  }
  int rc = gpio_write(pi_, pin_, 0);
  if (rc != 0) {RCLCPP_WARN(logger(), "gpio_write LOW failed on pin %d: rc=%d", pin_, rc);}
}

void GPIO::write_pwm(float frequency, float duty_cycle)
{
  if (direction_ != Direction::Output) {
    RCLCPP_WARN(logger(), "Illegal PWM write on pin %d", pin_);
    return;
  }
  int rc = set_PWM_frequency(pi_, pin_, static_cast<unsigned>(frequency));
  if (rc < 0) {
    RCLCPP_WARN(logger(), "set_PWM_frequency failed on pin %d: rc=%d", pin_, rc);
    return;
  }
  int duty_val = static_cast<int>(duty_cycle * 100.0f);
  rc = set_PWM_dutycycle(pi_, pin_, duty_val);
  if (rc != 0) {RCLCPP_WARN(logger(), "set_PWM_dutycycle failed on pin %d: rc=%d", pin_, rc);}
}

void GPIO::write_servo(int pulse_width)
{
  if (direction_ != Direction::Output) {
    RCLCPP_WARN(logger(), "Illegal servo write on pin %d", pin_);
    return;
  }
  int rc = set_servo_pulsewidth(pi_, pin_, pulse_width);
  if (rc != 0) {RCLCPP_WARN(logger(), "set_servo_pulsewidth failed on pin %d: rc=%d", pin_, rc);}
}

rclcpp::Logger GPIO::logger()
{
  return rclcpp::get_logger("gpio");
}
