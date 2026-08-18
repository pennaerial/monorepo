#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/time.pb.h>

#include "builtin_interfaces/msg/Time.h"
#include "sensor_msgs/msg/Imu.h"
#include "std_msgs/msg/Header.h"
#include "geometry_msgs/msg/Quaternion.h"
#include "geometry_msgs/msg/Vector3.h"


/**
 * @brief Finds the value of a key in a Gazebo message header's data fields.
 *
 * @param header Gazebo message header containing the data fields to search.
 * @param key Key whose associated value should be returned.
 * @return The value associated with @p key, or nullptr if the key is not found.
 */
const char* find_gz_header_data(const gz::msgs::Header& header, const std::string& key);

/**
 * @brief Copies the data from a Gazebo Float_V message into a double array.
 *
 * @param gz_msg Gazebo Float_V message containing the source data.
 * @param arr Destination double array to copy the data into.
 * @param N Number of elements in the destination DDS array.
 */
template<std::size_t N>
void gz_float_v_cpy(const gz::msgs::Float_V& gz_msg, double (&arr)[N])
{
  assert(gz_msg.data_size() == N);

  for (int i = 0; i < N; ++i) {
    arr[i] = static_cast<double>(gz_msg.data(i));
  }
}

/**
 * @brief Converts a Gazebo time message to a MicroXRCEDDS-generated time message.
 *
 * @param gz_msg Gazebo time message to convert.
 * @return The equivalent DDS time message.
 */
builtin_interfaces_msg_Time gz_to_dds(const gz::msgs::Time& gz_msg);

/**
 * @brief Converts a Gazebo IMU message to a MicroXRCEDDS-generated IMU message.
 *
 * @param gz_msg Gazebo IMU message to convert.
 * @return The equivalent DDS IMU message.
 */
sensor_msgs_msg_Imu gz_to_dds(const gz::msgs::IMU& gz_msg);

/**
 * @brief Converts a Gazebo header message to a MicroXRCEDDS-generated header message.
 *
 * @param gz_msg Gazebo header message to convert.
 * @return The equivalent DDS header message.
 */
std_msgs_msg_Header gz_to_dds(const gz::msgs::Header& gz_msg);

/**
 * @brief Converts a Gazebo Vector3d message to a MicroXRCEDDS-generated Vector3 message.
 *
 * @param gz_msg Gazebo Vector3d message to convert.
 * @return The equivalent DDS Vector3 message.
 */
geometry_msgs_msg_Vector3 gz_to_dds(const gz::msgs::Vector3d& gz_msg);

/**
 * @brief Converts a Gazebo Quaternion message to a MicroXRCEDDS-generated Quaternion message.
 *
 * @param gz_msg Gazebo Quaternion message to convert.
 * @return The equivalent DDS Quaternion message.
 */
geometry_msgs_msg_Quaternion gz_to_dds(const gz::msgs::Quaternion& gz_msg);
