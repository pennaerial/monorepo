#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <memory>
#include <algorithm>
#include "payload/gpio.hpp"

enum class MotorType {
    LEFT,
    RIGHT
};

class Motor {
    public:
        virtual ~Motor() = default;
        virtual void set_speed(float speed) = 0;
        virtual void forward(float duty) = 0;
        virtual void reverse(float duty) = 0;
        virtual void coast() = 0;
        virtual void hard_brake() = 0;
};

class DRVMotor : public Motor {
    public:
        DRVMotor(int pi, int in1, int in2, int frequency, MotorType motor_type);
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
        SNMotor(int pi, int pwm_pin, int in1, int in2, int frequency, MotorType motor_type);
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
