#pragma once

#include "dds_client.hpp"
#include "encoder.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.hpp"
#include "topics.h"

class Payload {
public:
    void init();
    void update();
    void publish_sensor_debug();
    void motor_updates();

    static Payload* instance();

private:
    Payload();
    ~Payload() = default;

    Payload(const Payload&) = delete;
    Payload& operator=(const Payload&) = delete;
    Payload(Payload&&) = delete;
    Payload& operator=(Payload&&) = delete;

    DDSClient dds_client;
    drivers::IMU* imu;
    drivers::Encoder* encoders;

};