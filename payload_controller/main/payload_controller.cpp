
#include "dds_client.hpp"
#include "esp_log.h"
#include "imu.hpp"
#include "encoder.hpp"

const char* TAG{"APP_MAIN"};

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Hello world");

  drivers::IMU* imu = drivers::IMU::instance();
  imu->start();

  drivers::Encoder* encoders = drivers::Encoder::instance();
  encoders->start();

  DDSClient dds_client("127.0.0.1", "7777");
  dds_client.run();


  while (1) {
    dds_client.update(imu->get_latest());
    encoders->publish_motor_left(55);
    encoders->publish_motor_right(5);

    // Delays by 50 ms to avoid spamming
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
