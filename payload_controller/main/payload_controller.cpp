#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "payload.h"

#include "imu.hpp"

extern "C" void app_main(void)
{
  // Temporary for esp testing
  #if defined(CONFIG_IDF_TARGET_LINUX)
    DDSClient::instance().init();
  #else
    drivers::IMU* imu = drivers::IMU::instance();
    imu->start();
  #endif

  Payload* payload = Payload::instance();
  payload->init();

  while (1) {
    // Temporary for esp testing
    #if defined(CONFIG_IDF_TARGET_LINUX)
    DDSClient::instance().update();
    #else 
      imu->update();
    #endif

    payload->update();

    // Delays by 100 ms to avoid spamming
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
