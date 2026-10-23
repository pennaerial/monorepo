#include "pwmcontroller.hpp"
#include <algorithm>

float PWMController::command(float deltaT, float reading) {
    float pi = pi_controller.update(deltaT, reading);
    float ff = ff_controller.update(deltaT);

    return std::clamp(ff + pi, -1.0f, 1.0f);
}

void PWMController::change_set_point(float new_set_point) {
    pi_controller.new_set_point(new_set_point);
    ff_controller.new_set_point(new_set_point);
}

