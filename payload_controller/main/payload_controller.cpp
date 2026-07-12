#include <memory>

#include "esp_log.h"
#include "imu.hpp"

const char* tag{"APP_MAIN"};

extern "C" void app_main(void) {
    ESP_LOGI(tag, "Hello world");
    std::unique_ptr<imu::IMU> imu = imu::make_imu();
    imu->start();


    while (1) {
    }
}
