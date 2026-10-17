#include "icm20948.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "ICM20948";

namespace {

const uint8_t WHO_AM_I_REG = 0x00;
const uint8_t PWR_MGMT_1_REG = 0x06;
const uint8_t PWR_MGMT_2_REG = 0x07;

const uint8_t REG_BANK_SEL_REG = 0x7F;
const uint8_t BANK_0 = 0x00;
const uint8_t BANK_1 = 0x10;
const uint8_t BANK_2 = 0x20;
const uint8_t BANK_3 = 0x30;
}

namespace drivers
{

IMU* IMU::instance()
{
    static ICM20948 instance;
    return &instance;
}


void ICM20948::start() {
    // Apparently necessary as the I2C port is initially unpowered 
    gpio_reset_pin(GPIO_NUM_7);
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1); // 1 = High

    // Setup the I2C port
    i2c_mst_config = {
    .i2c_port = I2C_NUM_0, // Us defining it
    .sda_io_num = GPIO_NUM_3,
    .scl_io_num = GPIO_NUM_4,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,
    .trans_queue_depth = 0,
    .flags = {.enable_internal_pullup = true, .allow_pd = false}
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Add the IMU as a device
    i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x69, // The default address for the IMU
    .scl_speed_hz = 100000, // Can be raised to 400000 in the future because that is the IMUs speed
    .scl_wait_us = 0,
    .flags = {.disable_ack_check = false},
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    ESP_LOGI(TAG, "Established connection to ICM20948");


    // Wake IMU & Accelerometer / Gyro
    writeToRegister(PWR_MGMT_1_REG, 0x01); // Sets the sleep bit (bit 6) to 0 to turn it on

    writeToRegister(PWR_MGMT_2_REG, 0x00); // Sets the first 6 bits to 0 which enables all 3 axes of gyro & accelerometer
    

    // Implied that we are reading from bank 0 as its the default
    uint8_t who = readFromRegister(WHO_AM_I_REG);
    ESP_LOGI(TAG, "Who is the device?: %i", who);
}
    
void ICM20948::writeToRegister(uint8_t reg, uint8_t data) {
    const uint8_t wr_buffer[2] = {reg, data};
    // 2 indicates a 2 byte write, -1 indicates infinite timeout (ms)
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, wr_buffer, 2, -1));
}

uint8_t ICM20948::readFromRegister(uint8_t reg) {
    uint8_t rd_buffer;
    // First 1 indicates 1 byte write, Second 1 indicates 1 byte read, -1 indicates infinite timeout (ms)
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, &rd_buffer, 1, -1));

    return rd_buffer;
}

void ICM20948::switchBank(uint8_t bankNum) {
    
}

};
