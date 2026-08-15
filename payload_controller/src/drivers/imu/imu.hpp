#pragma once

#include <memory>
namespace imu {


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

    /**
     * @brief Starts the IMU. It should immediately start writing imu messages to the IMU access layer
     */
    virtual void start() = 0;
};

/**
 * @brief Creates an instance of IMU, which is implemented by whichever IMU driver .cpp file is selected at compile time
 */
std::unique_ptr<IMU> make_imu();
}  // namespace imu


