#pragma once

#include "feedforward.hpp"
#include "pid.hpp"

class PWMController
{
public:
  PWMController(float kP, float kI, float setPoint, float maxIntegral, float kF, float kV, float setPointScale)
      : pi_controller(PID(kP, 0, kI, setPoint, maxIntegral)),
        ff_controller(FeedForward(kF, kV, setPoint, setPointScale)),
        setPoint(setPoint)
  {
  }

  float command(float deltaT, float reading);
  void change_set_point(float new_set_point);

private:
  // PID controller for feedback control (reactive)
  PID pi_controller;
  // FF controller for feedforward control (predictive)
  FeedForward ff_controller;

  // The target set point
  float setPoint;
};