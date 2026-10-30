#include "feedforward.hpp"

#include <cmath>

// separate this from general pwm controller which combines ff with pid

float FeedForward::update(float deltaT)
{
  if (deltaT <= 0 || set_point == 0) {
    return 0;
  }

  return out_for_fric * std::copysign(1.0, set_point) + out_per_in * set_point / set_point_scale;
}

void FeedForward::new_set_point(float new_set_point)
{
  set_point = new_set_point;
}