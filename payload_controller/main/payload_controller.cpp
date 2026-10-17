#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "payload.h"

#include "imu.hpp"

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
    payload->update();

    DDSClient::instance().update();
    #else 
      // IMU stuff
    #endif

    // Delays by 100 ms to avoid spamming
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
