#pragma once
#include "imu.hpp"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

namespace drivers
{
    
class ICM20948 : public IMU {

public:
    void start() override;
    void switchBank(uint8_t bankNum);
    void writeToRegister(uint8_t reg, uint8_t data);
    uint8_t readFromRegister(uint8_t reg);

private:
    i2c_master_bus_config_t i2c_mst_config;
    i2c_master_bus_handle_t bus_handle; // ESP Device
    i2c_master_dev_handle_t dev_handle; // IMU Device
};

};