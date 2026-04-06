#include "payload/motor.hpp"

#include <cmath>

//DRVMotor
DRVMotor::DRVMotor(int handle, int in1, int in2, int frequency, MotorType motor_type) 
: handle_(handle),
  frequency_(frequency),
  in1_(handle, in1, Direction::Output),
  in2_(handle, in2, Direction::Output),
  motor_type_(motor_type) { }

void DRVMotor::set_speed(float speed) {
    float norm_speed = std::clamp<float>(speed, -1.0f, 1.0f);
    if (norm_speed != speed) {
        RCLCPP_WARN(rclcpp::get_logger("motor"), "Clamped speed from %f to %f", speed, norm_speed);
    }

    float pwm_duty_cycle = std::abs(norm_speed * 100.0f); //percentage
    if (norm_speed > 0) { //forward, set phase to 0
        forward(pwm_duty_cycle);
    } else if (norm_speed < 0){ //reverse
        reverse(pwm_duty_cycle);
    } else { //coast
        coast();
    }
}

void DRVMotor::forward(float duty) {
    switch(motor_type_){
        case MotorType::LEFT:
            in1_.write_low();
            in2_.write_pwm(frequency_, duty, 0, 0);
            break;
        case MotorType::RIGHT:
            in1_.write_pwm(frequency_, duty, 0, 0);
            in2_.write_low();
            break;
    }
}

void DRVMotor::reverse(float duty) {
    switch(motor_type_){
        case MotorType::LEFT:
            in1_.write_pwm(frequency_, duty, 0, 0);
            in2_.write_low();
            break;
        case MotorType::RIGHT:
            in1_.write_low();
            in2_.write_pwm(frequency_, duty, 0, 0);
            break;
    }
}

void DRVMotor::coast() {
    in1_.write_pwm(frequency_, 0, 0, 0);
    in2_.write_pwm(frequency_, 0, 0, 0);
}

void DRVMotor::hard_brake() {
    in1_.write_high();
    in2_.write_high();
}
    

//SNMotor

SNMotor::SNMotor(int handle, int pwm_pin, int in1, int in2, int frequency, MotorType motor_type): 
    handle_(handle), 
    frequency_(frequency), 
    pwm_(handle, pwm_pin, Direction::Output),
    in1_(handle, in1, Direction::Output),
    in2_(handle, in2, Direction::Output),
    motor_type_(motor_type) { }


void SNMotor::set_speed(float speed) { 
    float norm_speed = std::clamp<float>(speed, -1.0f, 1.0f);
    if (norm_speed != speed) {
        RCLCPP_WARN(rclcpp::get_logger("motor"), "Clamped speed from %f to %f", speed, norm_speed);
    }

    float pwm_duty_cycle = std::abs(norm_speed * 100.0f); //percentage
    if (norm_speed > 0) { //forward, set phase to 0
        forward(pwm_duty_cycle);
    } else if (norm_speed < 0){ //reverse
        reverse(pwm_duty_cycle);
    } else { //coast
        coast();
    }
}

void SNMotor::forward(float duty) { 
    in1_.write_high();
    in2_.write_low();
    pwm_.write_pwm(frequency_, duty, 0, 0);
}
void SNMotor::reverse(float duty) { 
    in1_.write_low();
    in2_.write_high();
    pwm_.write_pwm(frequency_, duty, 0, 0);
}

void SNMotor::coast() { 
    in1_.write_low();
    in2_.write_low();
    pwm_.write_pwm(frequency_, 0.0f, 0, 0);
}

void SNMotor::hard_brake() { 
    in1_.write_high();
    in2_.write_high();
    pwm_.write_pwm(frequency_, 0.0f, 0, 0);
}
