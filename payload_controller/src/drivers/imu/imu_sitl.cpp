#include "imu.hpp"

#include <gz/transport/Node.hh>
#include <memory>
#include "sim_runtime.hpp"

namespace imu {

    class IMU_SITL : public IMU {
        public:
            IMU_SITL() : sim_cfg_(sim::get_config()) {
                gz_node_ = std::make_shared<gz::transport::Node>();
            }

            void start() override {

            }

        private:
            std::shared_ptr<gz::transport::Node> gz_node_;
            const sim::SimConfig sim_cfg_;

    };

    std::shared_ptr<IMU> make_imu() {
        return std::make_shared<imu::IMU_SITL>();

    }

}
