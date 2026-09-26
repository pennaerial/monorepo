#include "imu.hpp"

#include "dds_client.hpp"

namespace drivers
{

void IMU::write_latest(const sensor_msgs_msg_Imu& msg)
{
  {
    util::StaticMutexGuard lock(mtx_);
    reading_ = msg;
  }

  // Queue the DDS write outside the reading_ lock so update() can drain freely.
  if (dds_client_ != nullptr && dds_topic_name_ != nullptr) {
    dds_client_->publish(dds_topic_name_, &msg);
  }
}

void IMU::set_dds_publisher(DDSClient* dds_client, const char* topic_name)
{
  dds_client_ = dds_client;
  dds_topic_name_ = topic_name;
}

sensor_msgs_msg_Imu IMU::get_latest()
{
  util::StaticMutexGuard lock(mtx_);
  return reading_;
}

}  // namespace drivers
