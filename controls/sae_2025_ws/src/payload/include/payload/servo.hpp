#ifndef SERVO_HPP
#define SERVO_HPP

#include "payload/gpio.hpp"
#include "rclcpp/rclcpp.hpp"

class Servo {
    public:
        Servo(int handle, int pin, int frequency, float pulse_min_u, float pulse_max_u);
        void degree_setpoint(float degree);
        void normalized_setpoint(float t);  // t in [0.0, 1.0] → pulse_min to pulse_max

    private:
        int handle_{};
        int frequency_{};
        GPIO servo_pin_;

        int angle_to_pulse(float degree);
        float pulse_min_u_;
        float pulse_max_u_;
        static rclcpp::Logger logger();
};


#endif
