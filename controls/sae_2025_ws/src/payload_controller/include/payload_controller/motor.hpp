#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "payload_controller/gpio.hpp"

/// Which side of the vehicle this motor drives.
enum class MotorType {
    LEFT,  ///< Left-side motor
    RIGHT  ///< Right-side motor
};

/// Abstract base class for all motor drivers.
class Motor {
    public:
        virtual ~Motor() = default;
        /// Set motor speed in [-1, 1] (negative = reverse).
        virtual void set_speed(float speed) = 0;
        /// Drive forward at the given PWM duty cycle [0, 1].
        virtual void forward(float duty) = 0;
        /// Drive in reverse at the given PWM duty cycle [0, 1].
        virtual void reverse(float duty) = 0;
        /// Let the motor coast (no braking force).
        virtual void coast() = 0;
        /// Apply electronic braking immediately.
        virtual void hard_brake() = 0;
};

/// Motor driver for the DRV8833 H-bridge IC.
class DRVMotor : public Motor {
    public:
        /// @param pi        pigpio instance handle
        /// @param in1       GPIO pin for IN1
        /// @param in2       GPIO pin for IN2
        /// @param frequency PWM frequency in Hz
        /// @param motor_type Which side this motor drives
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
