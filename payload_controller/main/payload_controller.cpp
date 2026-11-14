#include <cstdlib>

#include "ekf.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "payload.hpp"

#if defined(CONFIG_IDF_TARGET_LINUX)
#include "sitl_runtime.hpp"
#endif

namespace
{
// Larger FreeRTOS numbers mean higher priority. Paired devices share a priority.
constexpr UBaseType_t DDS_PRIORITY = 7;
constexpr UBaseType_t SENSOR_PRIORITY = 6;
constexpr UBaseType_t EKF_PRIORITY = 5;
constexpr UBaseType_t PWM_PRIORITY = 4;
constexpr UBaseType_t MOTOR_PRIORITY = 3;
constexpr UBaseType_t PAYLOAD_PRIORITY = 2;
static_assert(DDS_PRIORITY < configMAX_PRIORITIES);

struct PeriodicTask {
  const char* name;
  void (*update)();
  uint32_t period_ms;
  UBaseType_t priority;
  uint32_t stack_bytes;
};

// Constructed before tasks start and accessed only by the EKF task afterwards.
EKF ekf;

void periodic_task(void* arg)
{
  const auto& task = *static_cast<const PeriodicTask*>(arg);
  const TickType_t ticks = pdMS_TO_TICKS(task.period_ms);
  const TickType_t period = ticks > 0 ? ticks : 1;
  TickType_t wake = xTaskGetTickCount();
  while (true) {
    task.update();
    // If an update overruns, block for one tick instead of spinning to catch up.
    if (xTaskDelayUntil(&wake, period) == pdFALSE) {
      vTaskDelay(1);
      wake = xTaskGetTickCount();
    }
  }
}

void dds_update()
{
  // Transport/session initialization belongs to the DDS task as well.
  static bool started = false;
  if (!started) {
    DDSClient::instance().init();
    started = true;
  }
  DDSClient::instance().update();
}

// Shared task layout for Linux SITL and ESP hardware. ESP-IDF stack sizes are bytes.
PeriodicTask tasks[] = {
    {"imu", [] { drivers::IMU::instance()->update(); }, 10, SENSOR_PRIORITY, 4096},
    {"encoder_left", [] { drivers::Encoder::instance(Side::LEFT).update(); }, 10, SENSOR_PRIORITY, 4096},
    {"encoder_right", [] { drivers::Encoder::instance(Side::RIGHT).update(); }, 10, SENSOR_PRIORITY, 4096},
    {"ekf", [] { ekf.update(); }, 10, EKF_PRIORITY, 32768},
    {"pwm", [] { Payload::instance()->motor_updates(); }, 10, PWM_PRIORITY, 4096},
    {"motor_left", [] { Payload::instance()->motor_output(Side::LEFT); }, 10, MOTOR_PRIORITY, 4096},
    {"motor_right", [] { Payload::instance()->motor_output(Side::RIGHT); }, 10, MOTOR_PRIORITY, 4096},
    {"payload", [] { Payload::instance()->update(); }, 100, PAYLOAD_PRIORITY, 8192},
    // Start DDS last so its agent handshake cannot delay creation of other workers.
    {"dds", dds_update, 10, DDS_PRIORITY, 8192},
};
}  // namespace

extern "C" void app_main(void)
{
#if defined(CONFIG_IDF_TARGET_LINUX)
  sitl::initialize_transport();
#endif

  // Initialize devices serially (the motors share a PWM timer) and bind DDS callbacks
  // before any worker can access them. Both targets run the full controller.
  Payload::instance()->init();
  ekf.init();
  for (auto& task : tasks) {
    if (xTaskCreate(periodic_task, task.name, task.stack_bytes, &task, task.priority, nullptr) != pdPASS) {
      ESP_LOGE("Tasks", "Failed to create %s task", task.name);
      std::abort();
    }
  }
}
