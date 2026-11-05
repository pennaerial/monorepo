#pragma once

class FeedForward
{
public:
  FeedForward(float out_for_fric, float out_per_in, float set_point, float set_point_scale)
      : out_for_fric(out_for_fric), out_per_in(out_per_in), set_point(set_point), set_point_scale(set_point_scale)
  {
  }

  // Generates a command based on an input delta time
  float update(float deltaT);
  // Updates the setpoint
  void new_set_point(float new_set_point);

private:
  // Quantity of output to compensate for friction
  float out_for_fric;
  // Quantity of output per amount of input (setpoint)
  float out_per_in;

  // The set point
  float set_point;
  // Measured speed at full duty under the calibration load, not a speed command.
  float set_point_scale;
};
