#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

namespace drivers::imu_hw_config
{

// Payload-controller wiring: GPIO 7 drives the load-switch enable for the IMU rail.
inline constexpr gpio_num_t POWER_ENABLE_GPIO = GPIO_NUM_7;
// Payload-controller wiring: the LSM6DSO SDA signal is connected to GPIO 3.
inline constexpr gpio_num_t I2C_SDA_GPIO = GPIO_NUM_3;
// Payload-controller wiring: the LSM6DSO SCL signal is connected to GPIO 4.
inline constexpr gpio_num_t I2C_SCL_GPIO = GPIO_NUM_4;
// The IMU is the only device currently assigned to ESP32-S3 I2C controller 0.
inline constexpr i2c_port_num_t I2C_PORT = I2C_NUM_0;

// ESP-IDF's recommended default filter value for rejecting short I2C glitches.
inline constexpr uint8_t I2C_GLITCH_IGNORE_COUNT = 7;
// Allow the sensor rail to settle after it is enabled before starting I2C traffic.
inline constexpr uint32_t POWER_SETTLE_TIME_MS = 10;
// Poll at 100 Hz to match the sensor's configured 104 Hz output data rate.
inline constexpr uint32_t SAMPLE_PERIOD_MS = 10;
// Print approximately eight samples per second without flooding the serial console.
inline constexpr uint32_t SAMPLES_PER_LOG = 12;
// ESP-IDF measures task stack size in bytes rather than FreeRTOS stack words.
inline constexpr uint32_t READ_TASK_STACK_SIZE_BYTES = 4096;
// Keep sensor acquisition above the idle task while leaving higher priorities available for control work.
inline constexpr uint32_t READ_TASK_PRIORITY = 5;
inline constexpr char READ_TASK_NAME[] = "imu_read";
// ROS frame attached to measurements from the physical IMU.
inline constexpr char FRAME_ID[] = "imu_link";

}  // namespace drivers::imu_hw_config
