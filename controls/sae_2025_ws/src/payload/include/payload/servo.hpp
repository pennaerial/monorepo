#ifndef SERVO_HPP
#define SERVO_HPP

#include "payload/gpio.hpp"
#include "rclcpp/rclcpp.hpp"

class Servo {
    public:
        Servo(int pin, int frequency, float pulse_min_u, float pulse_max_u);
        void degree_setpoint(float degree);
        void normalized_setpoint(float t);

    
    private:
        int frequency_{};
        GPIO servo_pin_;
        float pulse_min_u_{};
        float pulse_max_u_{};

        int angle_to_pulse(float degree);
        static rclcpp::Logger logger();
};


#endif
