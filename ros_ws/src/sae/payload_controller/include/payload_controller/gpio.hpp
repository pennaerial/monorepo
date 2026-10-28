#ifndef GPIO_HPP
#define GPIO_HPP

#include <rclcpp/rclcpp.hpp>

enum class Direction {
  Input,
  Output
};

class GPIO
{
public:
  GPIO(int pi, int pin, Direction direction);
  void write_high();
  void write_low();
  void write_pwm(float frequency, float duty_cycle);
  void write_servo(int pulse_width);

private:
  int pi_{};
  int pin_{};
  Direction direction_;

  static rclcpp::Logger logger();
};

#endif
