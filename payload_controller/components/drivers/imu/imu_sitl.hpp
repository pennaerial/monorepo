#pragma once

#include <cstddef>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>

#include "imu.hpp"
#include "sitl_runtime.hpp"

namespace drivers {

class IMU_SITL : public IMU {
   public:
    IMU_SITL();

    void start() override;

   private:
    /**
     * @brief Writes the IMU topic to a C-style char array.
     *
     * @param[out] buf Buffer that the topic string is written to.
     * @param size Size of the buffer. Use sizeof().
     */
    void make_imu_topic(char* buf, std::size_t size);

    /**
     * @brief Callback function for the IMU topic.
     * Runs in a gz background thread not managed by FreeRTOS/Linux/POSIX
     * simulator.
     *
     * @param gz_msg Gazebo IMU message.
     */
    void on_imu_msg(const gz::msgs::IMU& gz_msg);

    /// gz node instance for communicating with gazebo
    gz::transport::Node gz_node_;
    /// Contains Simulation Configuration (e.g. entity and world name)
    const sitl::SimConfig sitl_config_;
};

}  // namespace imu
