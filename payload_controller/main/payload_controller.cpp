#include "esp_log.h"

const char* tag { "APP_MAIN" };

extern "C"
void app_main(void)
{
    ESP_LOGI(tag, "Hello world");
}
