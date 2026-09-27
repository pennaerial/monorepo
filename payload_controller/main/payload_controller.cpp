
#include <cstring>

#include "dds_client.hpp"
#include "encoder.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.hpp"
#include "payload.h"

const char* TAG{"APP_MAIN"};

extern "C" void app_main(void)
{
  Payload* payload = Payload::instance();
  payload.init();

  while (1) {
    payload.update();

    // Delays by 100 ms to avoid spamming
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
