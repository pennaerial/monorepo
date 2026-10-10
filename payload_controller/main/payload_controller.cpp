#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "payload.h"

extern "C" void app_main(void)
{
  DDSClient::instance().init();

  Payload* payload = Payload::instance();
  payload->init();

  while (1) {
    payload->update();

    DDSClient::instance().update();

    // Delays by 100 ms to avoid spamming
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
