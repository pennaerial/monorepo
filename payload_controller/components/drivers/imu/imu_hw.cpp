#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.hpp"
#include "imu_hw_config.hpp"
#include "lsm6dso.hpp"

static const char* TAG = "IMU_HW";

namespace drivers
{

class IMU_HW : public IMU
{
public:
  void start() override
  {
    if (!enable_sensor_power() || !initialize_i2c_bus()) {
      return;
    }

    const esp_err_t result = sensor_.initialize(bus_);
    if (result != ESP_OK) {
      ESP_LOGE(
          TAG, "Failed to initialize LSM6DSO at 0x%02X: %s (WHO_AM_I=0x%02X)", LSM6DSO::I2C_ADDRESS,
          esp_err_to_name(result), sensor_.detected_device_id()
      );
      return;
    }

    ESP_LOGI(TAG, "LSM6DSO detected at 0x%02X", LSM6DSO::I2C_ADDRESS);
    if (xTaskCreate(
            read_task, imu_hw_config::READ_TASK_NAME, imu_hw_config::READ_TASK_STACK_SIZE_BYTES, this,
            imu_hw_config::READ_TASK_PRIORITY, nullptr
        ) != pdPASS) {
      ESP_LOGE(TAG, "Failed to create IMU read task");
    }
  }

private:
  bool enable_sensor_power()
  {
    if (gpio_set_direction(imu_hw_config::POWER_ENABLE_GPIO, GPIO_MODE_OUTPUT) != ESP_OK ||
        gpio_set_level(imu_hw_config::POWER_ENABLE_GPIO, 1) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to enable IMU power on GPIO %d", imu_hw_config::POWER_ENABLE_GPIO);
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(imu_hw_config::POWER_SETTLE_TIME_MS));
    return true;
  }

  bool initialize_i2c_bus()
  {
    const i2c_master_bus_config_t bus_config{
        .i2c_port = imu_hw_config::I2C_PORT,
        .sda_io_num = imu_hw_config::I2C_SDA_GPIO,
        .scl_io_num = imu_hw_config::I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = imu_hw_config::I2C_GLITCH_IGNORE_COUNT,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        // Internal pull-ups support bench wiring; the flight board should still provide external I2C pull-ups.
        .flags = {.enable_internal_pullup = true, .allow_pd = false},
    };
    const esp_err_t result = i2c_new_master_bus(&bus_config, &bus_);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(result));
      return false;
    }
    return true;
  }

  static void read_task(void* arg)
  {
    static_cast<IMU_HW*>(arg)->read_loop();
  }

  void read_loop()
  {
    uint32_t samples_since_log = 0;
    TickType_t next_wake_time = xTaskGetTickCount();
    while (true) {
      LSM6DSOSample sample{};
      const esp_err_t result = sensor_.read_sample(sample);
      if (result == ESP_OK) {
        const sensor_msgs_msg_Imu msg = make_imu_message(sample);
        write_latest(msg);

        if (++samples_since_log >= imu_hw_config::SAMPLES_PER_LOG) {
          samples_since_log = 0;
          ESP_LOGI(
              TAG, "accel [m/s^2] %.3f %.3f %.3f gyro [rad/s] %.3f %.3f %.3f", msg.linear_acceleration.x,
              msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y,
              msg.angular_velocity.z
          );
        }
      } else {
        ESP_LOGE(TAG, "Failed to read LSM6DSO: %s", esp_err_to_name(result));
      }
      // Delay against the previous wake time so I2C transaction time does not accumulate as sampling drift.
      xTaskDelayUntil(&next_wake_time, pdMS_TO_TICKS(imu_hw_config::SAMPLE_PERIOD_MS));
    }
  }

  static sensor_msgs_msg_Imu make_imu_message(const LSM6DSOSample& sample)
  {
    sensor_msgs_msg_Imu msg{};
    // No wall-clock source exists yet, so this stamp is monotonic time since ESP32 boot.
    constexpr int64_t MICROSECONDS_PER_SECOND = 1000000;
    constexpr int64_t NANOSECONDS_PER_MICROSECOND = 1000;
    const int64_t now_us = esp_timer_get_time();
    msg.header.stamp.sec = static_cast<int32_t>(now_us / MICROSECONDS_PER_SECOND);
    msg.header.stamp.nanosec = static_cast<uint32_t>((now_us % MICROSECONDS_PER_SECOND) * NANOSECONDS_PER_MICROSECOND);
    std::snprintf(msg.header.frame_id, sizeof(msg.header.frame_id), "%s", imu_hw_config::FRAME_ID);

    // A leading -1 follows sensor_msgs/Imu convention and marks orientation as unavailable.
    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity.x = sample.angular_velocity_rad_s[0];
    msg.angular_velocity.y = sample.angular_velocity_rad_s[1];
    msg.angular_velocity.z = sample.angular_velocity_rad_s[2];
    msg.linear_acceleration.x = sample.linear_acceleration_m_s2[0];
    msg.linear_acceleration.y = sample.linear_acceleration_m_s2[1];
    msg.linear_acceleration.z = sample.linear_acceleration_m_s2[2];
    return msg;
  }

  i2c_master_bus_handle_t bus_{nullptr};
  LSM6DSO sensor_;
};

IMU* IMU::instance()
{
  static IMU_HW instance;  // only instantiated once because static
  return &instance;
}

}  // namespace drivers
