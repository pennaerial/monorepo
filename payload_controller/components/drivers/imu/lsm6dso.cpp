#include "lsm6dso.hpp"

#include <cstddef>

namespace
{

// LSM6DSO register addresses from ST datasheet DS12140.
constexpr uint8_t WHO_AM_I_REGISTER = 0x0F;
constexpr uint8_t ACCELEROMETER_CONTROL_REGISTER = 0x10;
constexpr uint8_t GYROSCOPE_CONTROL_REGISTER = 0x11;
constexpr uint8_t COMMON_CONTROL_REGISTER = 0x12;
constexpr uint8_t GYROSCOPE_OUTPUT_START_REGISTER = 0x22;

// CTRL1_XL: 104 Hz output data rate and +/-2 g full scale.
constexpr uint8_t ACCELEROMETER_104_HZ_2_G = 0x40;
// CTRL2_G: 104 Hz output data rate and +/-250 dps full scale.
constexpr uint8_t GYROSCOPE_104_HZ_250_DPS = 0x40;
// CTRL3_C: block data updates until both bytes are read and auto-increment register addresses.
constexpr uint8_t BLOCK_DATA_UPDATE_AND_AUTO_INCREMENT = 0x44;

// Fast-mode I2C is supported by the LSM6DSO and keeps each sample transfer short.
constexpr uint32_t I2C_CLOCK_HZ = 400000;
// Bound each blocking ESP-IDF I2C transaction so a missing sensor cannot stall the task indefinitely.
constexpr int I2C_TRANSACTION_TIMEOUT_MS = 100;
// One sample contains three 16-bit gyro axes followed by three 16-bit accelerometer axes.
constexpr size_t RAW_SAMPLE_SIZE_BYTES = 12;
constexpr size_t AXIS_COUNT = 3;
constexpr size_t BYTES_PER_AXIS = 2;
constexpr size_t ACCELEROMETER_BYTE_OFFSET = 6;

// At +/-2 g, the LSM6DSO sensitivity is 0.061 mg/LSB. ROS uses m/s^2.
constexpr double ACCELEROMETER_METERS_PER_SECOND_SQUARED_PER_LSB = 0.000061 * 9.80665;
// At +/-250 dps, sensitivity is 8.75 mdps/LSB. ROS uses radians/second.
constexpr double GYROSCOPE_RADIANS_PER_SECOND_PER_LSB = 0.00875 * 3.14159265358979323846 / 180.0;

int16_t decode_little_endian_axis(const uint8_t* bytes)
{
  return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8));
}

}  // namespace

namespace drivers
{

esp_err_t LSM6DSO::initialize(i2c_master_bus_handle_t bus)
{
  const i2c_device_config_t device_config{
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = I2C_ADDRESS,
      .scl_speed_hz = I2C_CLOCK_HZ,
      .scl_wait_us = 0,
      .flags = {.disable_ack_check = false},
  };

  esp_err_t result = i2c_master_bus_add_device(bus, &device_config, &device_);
  if (result != ESP_OK) {
    return result;
  }

  result = read_register(WHO_AM_I_REGISTER, detected_device_id_);
  if (result != ESP_OK) {
    remove_device();
    return result;
  }
  if (detected_device_id_ != EXPECTED_DEVICE_ID) {
    remove_device();
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Enable coherent multi-byte reads before starting either sensor output.
  result = write_register(COMMON_CONTROL_REGISTER, BLOCK_DATA_UPDATE_AND_AUTO_INCREMENT);
  if (result == ESP_OK) {
    result = write_register(ACCELEROMETER_CONTROL_REGISTER, ACCELEROMETER_104_HZ_2_G);
  }
  if (result == ESP_OK) {
    result = write_register(GYROSCOPE_CONTROL_REGISTER, GYROSCOPE_104_HZ_250_DPS);
  }
  if (result != ESP_OK) {
    remove_device();
  }
  return result;
}

esp_err_t LSM6DSO::read_sample(LSM6DSOSample& sample)
{
  // Auto-increment makes 0x22-0x2D one burst: gyro XYZ first, then accelerometer XYZ.
  uint8_t raw[RAW_SAMPLE_SIZE_BYTES];
  const esp_err_t result = read_registers(GYROSCOPE_OUTPUT_START_REGISTER, raw, sizeof(raw));
  if (result != ESP_OK) {
    return result;
  }

  for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
    const size_t gyro_offset = axis * BYTES_PER_AXIS;
    const size_t accel_offset = ACCELEROMETER_BYTE_OFFSET + gyro_offset;
    sample.angular_velocity_rad_s[axis] =
        decode_little_endian_axis(&raw[gyro_offset]) * GYROSCOPE_RADIANS_PER_SECOND_PER_LSB;
    sample.linear_acceleration_m_s2[axis] =
        decode_little_endian_axis(&raw[accel_offset]) * ACCELEROMETER_METERS_PER_SECOND_SQUARED_PER_LSB;
  }
  return ESP_OK;
}

uint8_t LSM6DSO::detected_device_id() const
{
  return detected_device_id_;
}

esp_err_t LSM6DSO::write_register(uint8_t reg, uint8_t value)
{
  const uint8_t data[]{reg, value};
  return i2c_master_transmit(device_, data, sizeof(data), I2C_TRANSACTION_TIMEOUT_MS);
}

esp_err_t LSM6DSO::read_register(uint8_t reg, uint8_t& value)
{
  return read_registers(reg, &value, 1);
}

esp_err_t LSM6DSO::read_registers(uint8_t start_reg, uint8_t* data, size_t size)
{
  return i2c_master_transmit_receive(device_, &start_reg, sizeof(start_reg), data, size, I2C_TRANSACTION_TIMEOUT_MS);
}

void LSM6DSO::remove_device()
{
  if (device_ != nullptr) {
    i2c_master_bus_rm_device(device_);
    device_ = nullptr;
  }
}

}  // namespace drivers
