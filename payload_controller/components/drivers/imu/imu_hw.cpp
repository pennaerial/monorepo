#include <cmath>
#include <cstdint>
#include <cstring>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.hpp"

static const char* TAG = "IMU_HW";

namespace drivers
{

class IMU_HW : public IMU
{
public:
  void start() override
  {
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    const i2c_master_bus_config_t bus_config{
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_3,
        .scl_io_num = GPIO_NUM_4,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true, .allow_pd = false},
    };
    if (i2c_new_master_bus(&bus_config, &bus_) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize I2C bus");
      return;
    }

    const i2c_device_config_t device_config{
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDRESS,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags = {.disable_ack_check = false},
    };
    if (i2c_master_bus_add_device(bus_, &device_config, &device_) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to add LSM6DSO at 0x%02X", ADDRESS);
      return;
    }

    uint8_t who_am_i = 0;
    if (!read_register(WHO_AM_I, &who_am_i) || who_am_i != WHO_AM_I_VALUE) {
      ESP_LOGE(TAG, "Unexpected WHO_AM_I value 0x%02X", who_am_i);
      return;
    }

    if (!write_register(CTRL3_C, 0x44) || !write_register(CTRL1_XL, 0x40) || !write_register(CTRL2_G, 0x40)) {
      ESP_LOGE(TAG, "Failed to configure LSM6DSO");
      return;
    }

    ESP_LOGI(TAG, "LSM6DSO detected at 0x%02X", ADDRESS);
    xTaskCreate(read_task, "imu_read", 4096, this, 5, nullptr);
  }

private:
  static constexpr uint8_t ADDRESS = 0x6B;
  static constexpr uint8_t WHO_AM_I = 0x0F;
  static constexpr uint8_t WHO_AM_I_VALUE = 0x6C;
  static constexpr uint8_t CTRL1_XL = 0x10;
  static constexpr uint8_t CTRL2_G = 0x11;
  static constexpr uint8_t CTRL3_C = 0x12;
  static constexpr uint8_t OUT_X_L_G = 0x22;
  static constexpr uint8_t OUT_X_L_XL = 0x28;
  static constexpr double ACCEL_SCALE = 0.000061 * 9.80665;
  static constexpr double GYRO_SCALE = 0.00875 * 3.14159265358979323846 / 180.0;

  bool write_register(uint8_t reg, uint8_t value)
  {
    const uint8_t data[]{reg, value};
    return i2c_master_transmit(device_, data, sizeof(data), 100) == ESP_OK;
  }

  bool read_register(uint8_t reg, uint8_t* value)
  {
    return i2c_master_transmit_receive(device_, &reg, 1, value, 1, 100) == ESP_OK;
  }

  bool read_vector(uint8_t reg, int16_t values[3])
  {
    uint8_t data[6];
    if (i2c_master_transmit_receive(device_, &reg, 1, data, sizeof(data), 100) != ESP_OK) {
      return false;
    }
    for (int i = 0; i < 3; ++i) {
      values[i] =
          static_cast<int16_t>(static_cast<uint16_t>(data[2 * i]) | (static_cast<uint16_t>(data[2 * i + 1]) << 8));
    }
    return true;
  }

  static void read_task(void* arg)
  {
    static_cast<IMU_HW*>(arg)->read_loop();
  }

  void read_loop()
  {
    unsigned log_divider = 0;
    while (true) {
      int16_t gyro[3];
      int16_t accel[3];
      if (read_vector(OUT_X_L_G, gyro) && read_vector(OUT_X_L_XL, accel)) {
        sensor_msgs_msg_Imu msg{};
        const int64_t now_us = esp_timer_get_time();
        msg.header.stamp.sec = static_cast<int32_t>(now_us / 1000000);
        msg.header.stamp.nanosec = static_cast<uint32_t>((now_us % 1000000) * 1000);
        std::strncpy(msg.header.frame_id, "imu_link", sizeof(msg.header.frame_id) - 1);
        msg.orientation_covariance[0] = -1.0;
        msg.angular_velocity.x = gyro[0] * GYRO_SCALE;
        msg.angular_velocity.y = gyro[1] * GYRO_SCALE;
        msg.angular_velocity.z = gyro[2] * GYRO_SCALE;
        msg.linear_acceleration.x = accel[0] * ACCEL_SCALE;
        msg.linear_acceleration.y = accel[1] * ACCEL_SCALE;
        msg.linear_acceleration.z = accel[2] * ACCEL_SCALE;
        write_latest(msg);

        if (++log_divider == 12) {
          log_divider = 0;
          ESP_LOGI(
              TAG, "accel [m/s^2] %.3f %.3f %.3f gyro [rad/s] %.3f %.3f %.3f", msg.linear_acceleration.x,
              msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y,
              msg.angular_velocity.z
          );
        }
      } else {
        ESP_LOGE(TAG, "Failed to read LSM6DSO");
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  i2c_master_bus_handle_t bus_{nullptr};
  i2c_master_dev_handle_t device_{nullptr};
};

IMU* IMU::instance()
{
  static IMU_HW instance;  // only instantiated once because static
  return &instance;
}

}  // namespace drivers
