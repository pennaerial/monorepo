#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.hpp"
#include "payload.hpp"


extern "C" void app_main(void)
{
// Temporary for esp testing
#if defined(CONFIG_IDF_TARGET_LINUX)
  DDSClient::instance().init();

  Payload* payload = Payload::instance();
  payload->init();
#else
  drivers::IMU* imu = drivers::IMU::instance();
  imu->start();
#endif

  while (1) {
// Temporary for esp testing
#if defined(CONFIG_IDF_TARGET_LINUX)
    DDSClient::instance().update();

    payload->update();
#else
    imu->update();
#endif

    // Delays by 100 ms to avoid spamming
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
