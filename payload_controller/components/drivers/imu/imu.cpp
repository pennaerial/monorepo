#include "imu.hpp"

namespace drivers
{

void IMU::write_latest(const sensor_msgs_msg_Imu& msg)
{
  util::StaticMutexGuard lock(mtx_);
  reading_ = msg;
}

sensor_msgs_msg_Imu IMU::get_latest()
{
  util::StaticMutexGuard lock(mtx_);
  return reading_;
}

}  // namespace drivers
