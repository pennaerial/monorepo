#include "imu_sitl.hpp"

#include <cstdio>

#include "convert.hpp"
#include "esp_log.h"
#include "sensor_msgs/msg/Imu.h"
#include "std_msgs/msg/Header.h"

static const char* TAG = "IMU_SITL";

namespace drivers
{


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
      buf, size, "/world/%s/model/%s/link/base_link/sensor/imu_sensor/imu", sitl_config_.gz_world, sitl_config_.gz_model
  );
}

void IMU_SITL::on_imu_msg(const gz::msgs::IMU& gz_msg)
{
  sensor_msgs_msg_Imu msg = gz_to_dds(gz_msg);
  util::StaticMutexGuard lock(pending_mutex_);
  pending_ = msg;
  pending_available_ = true;
}

void IMU_SITL::update()
{
  sensor_msgs_msg_Imu sample{};
  {
    util::StaticMutexGuard lock(pending_mutex_);
    if (!pending_available_) {
      return;
    }
    sample = pending_;
    pending_available_ = false;
  }
  write_latest(sample);
}

IMU* IMU::instance()
{
  static IMU_SITL instance;  // only instantiated once because static
  return &instance;
}

}  // namespace drivers
