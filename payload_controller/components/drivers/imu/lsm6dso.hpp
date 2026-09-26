#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "imu.hpp"

namespace drivers
{

struct LSM6DSOSample {
  double angular_velocity_rad_s[3];
  double linear_acceleration_m_s2[3];
};

class LSM6DSO : public IMU
{
public:
  void start() override;

  esp_err_t initialize(i2c_master_bus_handle_t bus);
  esp_err_t read_sample(LSM6DSOSample& sample);
  uint8_t detected_device_id() const;

  // The board ties SA0 high, selecting the 0x6B 7-bit I2C address.
  static constexpr uint8_t I2C_ADDRESS = 0x6B;
  // ST assigns 0x6C to the LSM6DSO WHO_AM_I register.
  static constexpr uint8_t EXPECTED_DEVICE_ID = 0x6C;

private:
  // Payload-controller wiring: GPIO 7 drives the load-switch enable for the IMU rail.
  static constexpr gpio_num_t POWER_ENABLE_GPIO = GPIO_NUM_7;
  // Payload-controller wiring: the LSM6DSO SDA signal is connected to GPIO 3.
  static constexpr gpio_num_t I2C_SDA_GPIO = GPIO_NUM_3;
  // Payload-controller wiring: the LSM6DSO SCL signal is connected to GPIO 4.
  static constexpr gpio_num_t I2C_SCL_GPIO = GPIO_NUM_4;
  // The IMU is the only device currently assigned to ESP32-S3 I2C controller 0.
  static constexpr i2c_port_num_t I2C_PORT = I2C_NUM_0;

  // ESP-IDF's recommended default filter value for rejecting short I2C glitches.
  static constexpr uint8_t I2C_GLITCH_IGNORE_COUNT = 7;
  // Allow the sensor rail to settle after it is enabled before starting I2C traffic.
  static constexpr uint32_t POWER_SETTLE_TIME_MS = 10;
  // Poll at 100 Hz to match the sensor's configured 104 Hz output data rate.
  static constexpr uint32_t SAMPLE_PERIOD_MS = 10;
  // Print approximately eight samples per second without flooding the serial console.
  static constexpr uint32_t SAMPLES_PER_LOG = 12;
  // ESP-IDF measures task stack size in bytes rather than FreeRTOS stack words.
  static constexpr uint32_t READ_TASK_STACK_SIZE_BYTES = 4096;
  // Keep sensor acquisition above the idle task while leaving higher priorities available for control work.
  static constexpr uint32_t READ_TASK_PRIORITY = 5;
  static constexpr char READ_TASK_NAME[] = "imu_read";
  // ROS frame attached to measurements from the physical IMU.
  static constexpr char FRAME_ID[] = "imu_link";

  bool enable_sensor_power();
  bool initialize_i2c_bus();
  static void read_task(void* arg);
  void read_loop();
  static sensor_msgs_msg_Imu make_imu_message(const LSM6DSOSample& sample);

  esp_err_t write_register(uint8_t reg, uint8_t value);
  esp_err_t read_register(uint8_t reg, uint8_t& value);
  esp_err_t read_registers(uint8_t start_reg, uint8_t* data, size_t size);
  void remove_device();

  i2c_master_bus_handle_t bus_{nullptr};
  i2c_master_dev_handle_t device_{nullptr};
  uint8_t detected_device_id_{0};
};

}  // namespace drivers
