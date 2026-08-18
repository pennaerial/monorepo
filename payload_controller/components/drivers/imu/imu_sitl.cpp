#include "imu_sitl.hpp"
#include "sensor_msgs/msg/Imu.h"
#include <cstdio>


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
  sensor_msgs_msg_Imu msg;

}

IMU* IMU::instance() {
  static IMU_SITL instance; // only instantiated once because static
  return &instance;
}

}  // namespace imu
