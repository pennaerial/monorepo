
#include "esp_log.h"
#include "imu.hpp"
#include "dds_client.hpp"

const char* TAG{"APP_MAIN"};

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Hello world");

  drivers::IMU* imu = drivers::IMU::instance();
  imu->start();

  DDSClient dds_client(TransportType::UDP, "127.0.0.1", "7777");
  dds_client.run();


  while (1) {
    dds_client.update();
  }
}
