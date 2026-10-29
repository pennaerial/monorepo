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
  // The current target set point
  float setPoint;

  // P coefficient
  float kP;
  // D coefficient
  float kD;
  // I coefficient
  float kI;

  // Limits the maximum amount of the integral term
  float integralMax;

  // Whether we have recieved a second reading
  bool secondReading = false;
  // Reading from the past time step
  float pastReading = 0;

  // Current integral term sum
  float i = 0;
  // Past integral term sum for anti-windup reset
  float pastI = 0;

  // Past derivative term for anti-windup reset
  float pastD = 0;
};