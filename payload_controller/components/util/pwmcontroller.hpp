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
  PID pi_controller;
  FeedForward ff_controller;

  float setPoint;
};