
#include "dds_client.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.hpp"
#include "payload_controller_config.hpp"

const char* TAG{"APP_MAIN"};

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Hello world");

  drivers::IMU* imu = drivers::IMU::instance();
  imu->start();

  DDSClient dds_client(payload_controller_config::DDS_AGENT_IP, payload_controller_config::DDS_AGENT_PORT);
  dds_client.run();


  while (1) {
    // IMU acquisition runs independently; publish the newest complete sample at a bounded rate.
    dds_client.update(imu->get_latest());
    vTaskDelay(pdMS_TO_TICKS(payload_controller_config::DDS_PUBLISH_PERIOD_MS));
  }
}
