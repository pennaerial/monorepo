#pragma once

class PID
{
public:
  PID(float kP, float kD, float kI, float setPoint, float integralMax)
      : setPoint(setPoint), kP(kP), kD(kD), kI(kI), integralMax(integralMax)
  {
  }

  // One direct iteration of PID algo, antiWindup allows for rerunning the update function and removing integral update
  float update(float deltaT, float reading, bool antiWindup);
  // Set a new target set point for the controller
  void new_set_point(float nSetPoint);

private:
  float setPoint;

  float kP;
  float kD;
  float kI;

  float integralMax;

  bool secondReading = false;
  float pastReading = 0;

  float i = 0;
  float pastI = 0;

  float pastD = 0;
};