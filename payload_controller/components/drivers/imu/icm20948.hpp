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
  void update() override;
  void getSensorData();
  void start() override;
  void switchBank(uint8_t bankNum);
  void writeToRegister(uint8_t reg, uint8_t data);
  uint8_t readFromRegister(uint8_t reg);
  void multiReadStartingAtRegister(uint8_t startReg, uint8_t* readBuffer, size_t count);

private:
  double tempC;
  double accel[3];
  double gyro[3];


  i2c_master_bus_config_t i2c_mst_config;
  i2c_master_bus_handle_t bus_handle;  // ESP Device
  i2c_master_dev_handle_t dev_handle;  // IMU Device
};

};  // namespace drivers
