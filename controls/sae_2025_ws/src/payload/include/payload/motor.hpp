#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <memory>
#include "payload/gpio.hpp"
#include <algorithm>


enum class MotorType {
    LEFT,
    RIGHT
};

// Abstract motor interface.
class Motor {
    public:
        // Motor(int handle, int in1, int in2, int frequency, MotorType motor_type);
        virtual void set_speed(float speed) = 0;// Normalized speed, -1.0 for max reverse, 1.0 for max forward
        virtual void forward(float duty) = 0; // forward/coast mode
        virtual void reverse(float duty) = 0; // reverse/coast mode
        virtual void coast() = 0;
        virtual void hard_brake() = 0;
    

};

class DRVMotor : public Motor {
    public:
        DRVMotor(int in1, int in2, int frequency, MotorType motor_type);
        void set_speed(float speed) override;
        void forward(float duty) override;
        void reverse(float duty) override;
        void coast() override;
        void hard_brake() override;

    private:
        int frequency_{};
        GPIO in1_;
        GPIO in2_;
        MotorType motor_type_;
};


class SNMotor : public Motor {
    public:
        SNMotor(int pwm_pin, int in1, int in2, int frequency, MotorType motor_type);
        void set_speed(float speed) override;
        void forward(float duty) override;
        void reverse(float duty) override;
        void coast() override;
        void hard_brake() override;
    
    private:
        int frequency_{};
        GPIO pwm_;
        GPIO in1_;
        GPIO in2_;
        MotorType motor_type_;
};

#endif
