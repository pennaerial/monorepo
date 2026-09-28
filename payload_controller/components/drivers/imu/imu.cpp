#include "imu.hpp"

namespace drivers
{

void IMU::write_latest(const sensor_msgs_msg_Imu& msg)
{
  {
    util::StaticMutexGuard lock(mtx_);
    reading_ = msg;
    ++update_count_;
  }
}

sensor_msgs_msg_Imu IMU::get_latest()
{
  util::StaticMutexGuard lock(mtx_);
  return reading_;
}

bool IMU::get_latest_if_updated(uint32_t& last_update_count, sensor_msgs_msg_Imu& msg)
{
  util::StaticMutexGuard lock(mtx_);
  if (update_count_ == last_update_count) {
    return false;
  }

  msg = reading_;
  last_update_count = update_count_;
  return true;
}

}  // namespace drivers
