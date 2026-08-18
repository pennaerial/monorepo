#include <cstdio>
#include "imu_sitl.hpp"
#include "convert.hpp"
#include "sensor_msgs/msg/Imu.h"
#include "std_msgs/msg/Header.h"


#include "esp_log.h"

static const char* TAG = "IMU_SITL";

namespace drivers {


IMU_SITL::IMU_SITL() : sitl_config_(sitl::get_config()) {}

void IMU_SITL::start()
{
  char topic[128];
  make_imu_topic(topic, sizeof(topic));

  ESP_LOGI(TAG, "Subscribing to %s", topic);
  gz_node_.Subscribe(topic, &IMU_SITL::on_imu_msg, this);
}

void IMU_SITL::make_imu_topic(char* buf, std::size_t size)
{
  std::snprintf(
    buf,
    size,
    "/world/%s/model/%s/link/base_link/sensor/imu_sensor/imu",
    sitl_config_.gz_world,
    sitl_config_.gz_model
  );
}

void IMU_SITL::on_imu_msg(const gz::msgs::IMU& gz_msg)
{
  ESP_LOGI(TAG, "on_imu_msg");

  sensor_msgs_msg_Imu msg = gz_to_dds(gz_msg);
  ESP_LOGI(TAG, "hdr stamp sec: %d", msg.header.stamp.sec);
  ESP_LOGI(TAG, "hdr stamp nsec: %d", msg.header.stamp.nanosec);
  ESP_LOGI(TAG, "hdr frame_id: %s", msg.header.frame_id);
  ESP_LOGI(TAG, "orientation x: %f", msg.orientation.x);
  ESP_LOGI(TAG, "orientation y: %f", msg.orientation.y);
  ESP_LOGI(TAG, "orientation z: %f", msg.orientation.z);
  ESP_LOGI(TAG, "orientation w: %f", msg.orientation.w);

  ESP_LOGI(TAG, "orientation covariance: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
    msg.orientation_covariance[0],
    msg.orientation_covariance[1],
    msg.orientation_covariance[2],
    msg.orientation_covariance[3],
    msg.orientation_covariance[4],
    msg.orientation_covariance[5],
    msg.orientation_covariance[6],
    msg.orientation_covariance[7],
    msg.orientation_covariance[8]);

  ESP_LOGI(TAG, "angular velocity x: %f", msg.angular_velocity.x);
  ESP_LOGI(TAG, "angular velocity y: %f", msg.angular_velocity.y);
  ESP_LOGI(TAG, "angular velocity z: %f", msg.angular_velocity.z);

  ESP_LOGI(TAG, "angular velocity covariance: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
    msg.angular_velocity_covariance[0],
    msg.angular_velocity_covariance[1],
    msg.angular_velocity_covariance[2],
    msg.angular_velocity_covariance[3],
    msg.angular_velocity_covariance[4],
    msg.angular_velocity_covariance[5],
    msg.angular_velocity_covariance[6],
    msg.angular_velocity_covariance[7],
    msg.angular_velocity_covariance[8]);

  ESP_LOGI(TAG, "linear acceleration x: %f", msg.linear_acceleration.x);
  ESP_LOGI(TAG, "linear acceleration y: %f", msg.linear_acceleration.y);
  ESP_LOGI(TAG, "linear acceleration z: %f", msg.linear_acceleration.z);

  ESP_LOGI(TAG, "linear acceleration covariance: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
    msg.linear_acceleration_covariance[0],
    msg.linear_acceleration_covariance[1],
    msg.linear_acceleration_covariance[2],
    msg.linear_acceleration_covariance[3],
    msg.linear_acceleration_covariance[4],
    msg.linear_acceleration_covariance[5],
    msg.linear_acceleration_covariance[6],
    msg.linear_acceleration_covariance[7],
    msg.linear_acceleration_covariance[8]);
}

IMU* IMU::instance() {
  static IMU_SITL instance; // only instantiated once because static
  return &instance;
}

}  // namespace drivers
