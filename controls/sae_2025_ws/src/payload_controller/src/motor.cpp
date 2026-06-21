#include "payload_controller/motor.hpp"

#include <cmath>

// DRVMotor

DRVMotor::DRVMotor(int pi, int in1, int in2, int frequency, MotorType motor_type)
: frequency_(frequency),
  in1_(pi, in1, Direction::Output),
  in2_(pi, in2, Direction::Output),
  motor_type_(motor_type) {}

void DRVMotor::set_speed(float speed) {
    float norm_speed = std::clamp<float>(speed, -1.0f, 1.0f);
    float pwm_duty_cycle = std::abs(norm_speed * 100.0f);
    if (norm_speed > 0) {
        forward(pwm_duty_cycle);
    } else if (norm_speed < 0) {
        reverse(pwm_duty_cycle);
    } else {
        coast();
    }
}

void DRVMotor::forward(float duty) {
    switch (motor_type_) {
        case MotorType::LEFT:
            in1_.write_low();
            in2_.write_pwm(frequency_, duty);
            break;
        case MotorType::RIGHT:
            in1_.write_pwm(frequency_, duty);
            in2_.write_low();
            break;
    }
}

void DRVMotor::reverse(float duty) {
    switch (motor_type_) {
        case MotorType::LEFT:
            in1_.write_pwm(frequency_, duty);
            in2_.write_low();
            break;
        case MotorType::RIGHT:
            in1_.write_low();
            in2_.write_pwm(frequency_, duty);
            break;
    }
}

void DRVMotor::coast() {
    in1_.write_pwm(frequency_, 0);
    in2_.write_pwm(frequency_, 0);
}

void DRVMotor::hard_brake() {
    in1_.write_high();
    in2_.write_high();
}


// SNMotor

SNMotor::SNMotor(int pi, int pwm_pin, int in1, int in2, int frequency, MotorType motor_type)
: frequency_(frequency),
  pwm_(pi, pwm_pin, Direction::Output),
  in1_(pi, in1, Direction::Output),
  in2_(pi, in2, Direction::Output),
  motor_type_(motor_type) {}

void SNMotor::set_speed(float speed) {
    float norm_speed = std::clamp<float>(speed, -1.0f, 1.0f);
    float pwm_duty_cycle = std::abs(norm_speed * 100.0f);
    if (norm_speed > 0) {
        forward(pwm_duty_cycle);
    } else if (norm_speed < 0) {
        reverse(pwm_duty_cycle);
    } else {
        coast();
    }
}

void SNMotor::forward(float duty) {
    in1_.write_high();
    in2_.write_low();
    pwm_.write_pwm(frequency_, duty);
}

void SNMotor::reverse(float duty) {
    in1_.write_low();
    in2_.write_high();
    pwm_.write_pwm(frequency_, duty);
}

void SNMotor::coast() {
    in1_.write_low();
    in2_.write_low();
    pwm_.write_pwm(frequency_, 0.0f);
}

void SNMotor::hard_brake() {
    in1_.write_high();
    in2_.write_high();
    pwm_.write_pwm(frequency_, 0.0f);
}
