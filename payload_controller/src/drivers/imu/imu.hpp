#ifndef IMU_HPP
#define IMU_HPP

#include <memory>
namespace imu {


    class IMU {
        public:
            virtual ~IMU() = default;
            virtual void start() = 0;
    };

    std::unique_ptr<IMU> make_imu();
}


#endif
