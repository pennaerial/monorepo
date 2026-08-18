#include "imu.hpp"
#include "esp_log.h"

static const char* TAG = "IMU_HW";

namespace drivers {

/**
 * @brief Stub real-hardware IMU implementation. Not wired to any sensor yet
 * — exists so non-"linux" targets (e.g. esp32s3) link.
 */
class IMU_HW : public IMU {
   public:
    void start() override {
        ESP_LOGW(TAG, "IMU_HW::start() is a stub, no sensor is actually read yet");
    }
};

IMU* IMU::instance() {
  static IMU_HW instance; // only instantiated once because static
  return &instance;
}

}  // namespace drivers
