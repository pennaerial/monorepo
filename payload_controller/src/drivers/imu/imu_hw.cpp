#include <memory>

#include "esp_log.h"
#include "imu.hpp"

static const char* TAG = "IMU_HW";

namespace imu {

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

std::unique_ptr<IMU> make_imu() {
    return std::make_unique<IMU_HW>();
}

}  // namespace imu
