#include "payload/servo.hpp"

Servo::Servo(int handle, int pin, int frequency, float pulse_min_u, float pulse_max_u) :
    handle_(handle),
    frequency_(frequency),
    servo_pin_(handle, pin, Direction::Output),
    pulse_min_u_(pulse_min_u),
    pulse_max_u_(pulse_max_u) {}

void Servo::degree_setpoint(float degree) {
    int pulse = angle_to_pulse(degree);
    RCLCPP_DEBUG(logger(), "Outputting pulse: %d", pulse);
    servo_pin_.write_servo(pulse, frequency_, 0, 0);
}


void Servo::normalized_setpoint(float t) {
    float clamped = std::clamp(t, 0.0f, 1.0f);
    int pulse = static_cast<int>(pulse_min_u_ + clamped * (pulse_max_u_ - pulse_min_u_));
    RCLCPP_DEBUG(logger(), "Normalized %.2f → pulse: %d", clamped, pulse);
    servo_pin_.write_servo(pulse, frequency_, 0, 0);
}

int Servo::angle_to_pulse(float degree) {
    // Pulse range for this servo: 1000 µs = 0°, 3000 µs = 180°
    float clamped = std::clamp(degree, 0.0f, 180.0f);
    return static_cast<int>(pulse_min_u_ + (clamped / 180.0f) * (pulse_max_u_ - pulse_min_u_));
}


rclcpp::Logger Servo::logger() {
    return rclcpp::get_logger("servo");
}
