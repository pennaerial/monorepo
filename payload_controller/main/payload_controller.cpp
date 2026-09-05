
#include "esp_log.h"
#include "imu.hpp"

const char* tag{"APP_MAIN"};

extern "C" void app_main(void)
{
  ESP_LOGI(tag, "Hello world");

  drivers::IMU* imu = drivers::IMU::instance();
  imu->start();


  while (1) {
  }
}
