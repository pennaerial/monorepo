#pragma once

#include "pid.hpp"
#include "feedforward.hpp"

class PWMController {
    public:
        PWMController(float kP, float kI, float setPoint, float kF, float kV) : pi_controller(PID(kP, 0, kI, setPoint)), ff_controller(FeedForward(kF, kV, setPoint)) {}

        float command(float deltaT, float reading);
        void change_set_point(float new_set_point);
    
    private:
        PID pi_controller;
        FeedForward ff_controller;
};