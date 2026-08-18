#include <gz/msgs/time.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/header.pb.h>

#include "builtin_interfaces/msg/Time.h"
#include "sensor_msgs/msg/Imu.h"
#include "std_msgs/msg/Header.h"
#include "geometry_msgs/msg/Quaternion.h"
#include "geometry_msgs/msg/Vector3.h"



inline builtin_interfaces_msg_Time gz_to_dds(const gz::msgs::Time& gz_msg) {
  builtin_interfaces_msg_Time dds_msg{};
  dds_msg.sec = gz_msg.sec();
  dds_msg.nanosec = static_cast<uint32_t>(gz_msg.nsec());
  return dds_msg;
}

inline const char* find_gz_header_data(const gz::msgs::Header& header, const std::string& key)
{
    for (const auto& d : header.data()) {
        if (d.key() == key && d.value_size() > 0) {
            return d.value(0).c_str();
        }
    }
    return nullptr;
}

inline std_msgs_msg_Header gz_to_dds(const gz::msgs::Header& gz_msg) {
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
