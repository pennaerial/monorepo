#pragma once
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "geometry_msgs/msg/Vector3.h"
#include "imu.hpp"

namespace drivers
{

class ICM20948 : public IMU
{
public:
  // The general update loop grabbing the sensor data and doing minor processing
  void update() override;
  // Actually fetching the sensor data from the registers on the IMU
  void get_sensor_data();
  // Setting up the I2C communication and enabling the IMU & its individual sensors
  void start() override;
  // Swaps the bank to which we are sending our reads / writes to
  void switch_bank(uint8_t bankNum);
  // Writes one byte of data to one specific register in the current selected bank
  void write_to_register(uint8_t reg, uint8_t data);
  // Reads one byte of data from one specific register in the current selected bank
  uint8_t read_from_register(uint8_t reg);
  // Reads <count> bytes of data starting at one register and getting the consecutive registers in the current bank
  void multi_read_starting_at_register(uint8_t startReg, uint8_t* readBuffer, size_t count);

private:
  double tempC;
  double accel[3];
  double gyro[3];


  i2c_master_bus_config_t i2c_mst_config;
  i2c_master_bus_handle_t bus_handle;  // ESP Device
  i2c_master_dev_handle_t dev_handle;  // IMU Device
};

};  // namespace drivers
