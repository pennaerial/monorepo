#include "imu.hpp"

namespace drivers {

void IMU::write(const sensor_msgs_msg_Imu& msg)
{
  util::StaticMutexGuard lock(mtx_);
  reading_ = msg;
}

sensor_msgs_msg_Imu IMU::latest()
{
  util::StaticMutexGuard lock(mtx_);
  return reading_;
}

}
