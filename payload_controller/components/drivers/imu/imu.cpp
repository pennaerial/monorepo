#include "imu.hpp"

namespace drivers
{

void IMU::write_latest(const sensor_msgs_msg_Imu& msg)
{
  {
    util::TaskMutexGuard lock(mtx_);
    reading_ = msg;
    ++update_count_;
    estimator_samples_.push(
        Sample{
            static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec,
            {msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z},
            {msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z}
        }
    );
  }
}

std::size_t IMU::read_samples(Sample* samples, std::size_t capacity, std::size_t& dropped)
{
  util::TaskMutexGuard lock(mtx_);
  dropped = estimator_samples_.take_dropped();
  std::size_t count = 0;
  while (count < capacity && estimator_samples_.pop(samples[count])) {
    ++count;
  }
  return count;
}

sensor_msgs_msg_Imu IMU::get_latest()
{
  util::TaskMutexGuard lock(mtx_);
  return reading_;
}

bool IMU::get_latest_if_updated(uint32_t& last_update_count, sensor_msgs_msg_Imu& msg)
{
  util::TaskMutexGuard lock(mtx_);
  if (update_count_ == last_update_count) {
    return false;
  }

  msg = reading_;
  last_update_count = update_count_;
  return true;
}

}  // namespace drivers
