#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

namespace drivers
{

// One synchronized six-axis sample converted to the SI units expected by sensor_msgs/Imu.
struct LSM6DSOSample {
  double angular_velocity_rad_s[3];
  double linear_acceleration_m_s2[3];
};

class LSM6DSO
{
public:
  // Registers the sensor on an existing I2C bus, verifies its identity, and enables both outputs.
  esp_err_t initialize(i2c_master_bus_handle_t bus);
  // Reads the gyro and accelerometer register block and converts all axes to SI units.
  esp_err_t read_sample(LSM6DSOSample& sample);
  // Exposes the last WHO_AM_I response so initialization failures can report the detected device.
  uint8_t detected_device_id() const;

  // The board ties SA0 high, selecting the 0x6B 7-bit I2C address.
  static constexpr uint8_t I2C_ADDRESS = 0x6B;
  // ST assigns 0x6C to the LSM6DSO WHO_AM_I register.
  static constexpr uint8_t EXPECTED_DEVICE_ID = 0x6C;

private:
  esp_err_t write_register(uint8_t reg, uint8_t value);
  esp_err_t read_register(uint8_t reg, uint8_t& value);
  esp_err_t read_registers(uint8_t start_reg, uint8_t* data, size_t size);
  void remove_device();

  i2c_master_dev_handle_t device_{nullptr};
  uint8_t detected_device_id_{0};
};

}  // namespace drivers
