#include "convert.hpp"
#include "geometry_msgs/msg/Quaternion.h"
#include "geometry_msgs/msg/Vector3.h"

const char* find_gz_header_data(const gz::msgs::Header& header, const std::string& key)
{
    for (const auto& d : header.data()) {
        if (d.key() == key && d.value_size() > 0) {
            return d.value(0).c_str();
        }
    }
    return nullptr;
}

builtin_interfaces_msg_Time gz_to_dds(const gz::msgs::Time& gz_msg) {
  builtin_interfaces_msg_Time dds_msg{};
  dds_msg.sec = gz_msg.sec();
  dds_msg.nanosec = static_cast<uint32_t>(gz_msg.nsec());
  return dds_msg;
}

sensor_msgs_msg_Imu gz_to_dds(const gz::msgs::IMU& gz_msg)
{
  sensor_msgs_msg_Imu dds_msg{};
  dds_msg.header = gz_to_dds(gz_msg.header());

  dds_msg.orientation = gz_to_dds(gz_msg.orientation());
  gz_float_v_cpy(gz_msg.orientation_covariance(), dds_msg.orientation_covariance);

  dds_msg.angular_velocity = gz_to_dds(gz_msg.angular_velocity());
  gz_float_v_cpy(gz_msg.angular_velocity_covariance(), dds_msg.angular_velocity_covariance);

  dds_msg.linear_acceleration = gz_to_dds(gz_msg.linear_acceleration());
  gz_float_v_cpy(gz_msg.linear_acceleration_covariance(), dds_msg.linear_acceleration_covariance);

  return dds_msg;
}

std_msgs_msg_Header gz_to_dds(const gz::msgs::Header& gz_msg) {
  std_msgs_msg_Header dds_msg{};

  if (gz_msg.has_stamp()) {
      dds_msg.stamp = gz_to_dds(gz_msg.stamp());
  }

  const char* frame_id = find_gz_header_data(gz_msg, "frame_id");
  if (frame_id) {
    strncpy(dds_msg.frame_id, frame_id, sizeof(dds_msg.frame_id) - 1);
  }
  return dds_msg;
}

geometry_msgs_msg_Vector3 gz_to_dds(const gz::msgs::Vector3d& gz_msg)
{
  geometry_msgs_msg_Vector3 dds_msg{};
  dds_msg.x = gz_msg.x();
  dds_msg.y = gz_msg.y();
  dds_msg.z = gz_msg.z();
  return dds_msg;
}

geometry_msgs_msg_Quaternion gz_to_dds(const gz::msgs::Quaternion& gz_msg)
{
  geometry_msgs_msg_Quaternion dds_msg{};
  dds_msg.x = gz_msg.x();
  dds_msg.y = gz_msg.y();
  dds_msg.z = gz_msg.z();
  dds_msg.w = gz_msg.w();
  return dds_msg;
}
