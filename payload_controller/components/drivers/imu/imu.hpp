#pragma once

#include "sensor_msgs/msg/Imu.h"
#include "static_mutex.hpp"

namespace drivers {

/**
 * @class IMU
 * @brief Abstract Interface for IMU sensor class. The point of this class is to give a clean interface so
 * the Payload Controller doesn't need to know where it gets sensor data from. This header is the only interface
 * the rest of the application needs, hence why there is no imu_sitl.hpp
 *
 */
class IMU {
   public:
    virtual ~IMU() = default;

    /// Starts the IMU. It should immediately start writing imu messages to internal buffer
    virtual void start() = 0;
    /// Writes a new incoming imu reading to
    void write_latest(const sensor_msgs_msg_Imu& msg);
    /// returns the latest imu reading
    sensor_msgs_msg_Imu get_latest();
    /// Gets singleton instance of imu implementation (sitl, hardware). implemented in backend .cpp files, not imu.cpp
    static IMU* instance();

   protected:
    IMU() = default; //prevent public instantiation

   private:
    sensor_msgs_msg_Imu reading_;
    util::StaticMutex mtx_;


   private:
    /// Implemented once per backend (imu_sitl.cpp / imu_hw.cpp).
    /// Constructs the concrete backend instance.
    static IMU& create();
};
}  // namespace drivers


