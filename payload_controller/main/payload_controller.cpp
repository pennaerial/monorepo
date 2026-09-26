
#include "dds_client.hpp"
#include "encoder.hpp"
#include "esp_log.h"
#include "imu.hpp"

#include <cstring>

const char* TAG{"APP_MAIN"};

void testCallback(const Topic& topic, const void* msg, uint16_t length, void* args)
{
  (void)length;
  (void)args;

  if (std::strcmp(topic.type_name, "sensor_msgs::msg::dds_::Imu_") != 0) {
    ESP_LOGW(TAG, "Unexpected DDS callback type: %s", topic.type_name);
    return;
  }

  const sensor_msgs_msg_Imu* imu_msg = static_cast<const sensor_msgs_msg_Imu*>(msg);
  ESP_LOGI(TAG, "DDS IMU callback orientation: [%f, %f, %f, %f]",
           imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
}

extern "C" void app_main(void)
{
  drivers::IMU* imu = drivers::IMU::instance();
  imu->start();

  drivers::Encoder* encoders = drivers::Encoder::instance();
  encoders->start();

  DDSClient dds_client("127.0.0.1", "7777");
  dds_client.init();

  if (!dds_client.set_reader_callback("rt/imu", &testCallback, nullptr)) {
    ESP_LOGW(TAG, "Failed to bind IMU DDS reader callback");
  }

  while (1) {
    sensor_msgs_msg_Imu imu_msg = imu->get_latest();
    dds_client.publish("IMU", &imu_msg);
    dds_client.update();

    encoders->publish_motor_left(10);
    encoders->publish_motor_right(5);

    // Delays by 50 ms to avoid spamming
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
