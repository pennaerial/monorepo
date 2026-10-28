#include "pwmcontroller.hpp"

#include <algorithm>

float PWMController::command(float deltaT, float reading)
{
  float pi = pi_controller.update(deltaT, reading, false);
  float ff = ff_controller.update(deltaT);

  float rawPWM = pi + ff;
  float error = setPoint - reading;

  // Check for integral term windup
  bool windup = (rawPWM >= 1.0f && error > 0.0f) || (rawPWM <= -1.0f && error < 0.0f);

  if (windup) {
    pi = pi_controller.update(deltaT, reading, true);
  }

  return std::clamp(ff + pi, -1.0f, 1.0f);
}

void PWMController::change_set_point(float new_set_point)
{
  pi_controller.new_set_point(new_set_point);
  ff_controller.new_set_point(new_set_point);

  setPoint = new_set_point;
}