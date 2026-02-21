#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <memory>
#include "payload/gpio.hpp"
#include <algorithm>


class Motor {
    public:
        Motor(int handle, int phase, int enable);
        void set_speed(float speed);// Normalized speed, -1.0 for max reverse, 1.0 for max forward
    

    private:
        int handle_{};
        std::unique_ptr<GPIO> phase_;   //binary phase_pin, 
        std::unique_ptr<GPIO> enable_;  //pwm only enable_pin


};


#endif