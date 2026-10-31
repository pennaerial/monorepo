#include "encoder.hpp"

namespace drivers
{

void Encoder::write_latest(const int64_t ticks)
{
  util::StaticMutexGuard lock(mtx_);
  ticks_ = ticks;
  ++update_count_;
}

void Encoder::write_latest(const int64_t ticks, const double angular_velocity_rad_s)
{
  util::StaticMutexGuard lock(mtx_);
  ticks_ = ticks;
  angular_velocity_rad_s_ = angular_velocity_rad_s;
  ++update_count_;
  ++velocity_update_count_;
}

int64_t Encoder::get_latest()
{
  util::StaticMutexGuard lock(mtx_);
  return ticks_;
}

bool Encoder::get_latest_if_updated(uint32_t& last_update_count, int64_t& ticks)
{
  util::StaticMutexGuard lock(mtx_);
  if (update_count_ == last_update_count) {
    return false;
  }

  ticks = ticks_;
  last_update_count = update_count_;
  return true;
}

double Encoder::get_latest_angular_velocity()
{
  util::StaticMutexGuard lock(mtx_);
  return angular_velocity_rad_s_;
}

bool Encoder::get_latest_angular_velocity_if_updated(uint32_t& last_update_count, double& angular_velocity_rad_s)
{
  util::StaticMutexGuard lock(mtx_);
  if (velocity_update_count_ == last_update_count) {
    return false;
  }

  angular_velocity_rad_s = angular_velocity_rad_s_;
  last_update_count = velocity_update_count_;
  return true;
}

}  // namespace drivers
