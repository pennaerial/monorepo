#ifndef IMU_HPP
#define IMU_HPP

#include <memory>
namespace imu {


    class IMU {
        public:
            virtual void start() = 0;
    };

    std::shared_ptr<IMU> make_imu();
}


#endif
