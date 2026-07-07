#include "imu.hpp"


#include <cstddef>
#include <gz/msgs/details/imu.pb.h>
#include <memory>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include "esp_log.h"
#include "sitl_runtime.hpp"

static const char* TAG = "IMU_SITL";

namespace imu {

    class IMU_SITL : public IMU {
        public:
            IMU_SITL() : sitl_cfg_(sitl::get_config()) { }

            void start() override {
                char topic[128];
                make_imu_topic(topic, sizeof(topic));
                ESP_LOGI(TAG, "Subscribing to %s", topic);
                node_.Subscribe(
                    topic,
                    &IMU_SITL::on_imu_msg,
                    this
                );
            }

            /**
             * @brief Writes the imu topic to a C-style char array
             *
             * @param[out] buf Buffer that the topic str is written to
             * @param size size of the buffer. Use sizeof()
             */
            void make_imu_topic(char* buf, size_t size) {
                snprintf(
                    buf,
                    size,
                    "/world/%s/model/%s/link/base_link/sensor/imu_sensor/imu",
                    sitl_cfg_.gz_world,
                    sitl_cfg_.gz_model
                );
            }

            /**
             * @brief callback function for imu topic. Runs in a gz background thread not managed by FreeRTOS Linux/POSIX simulator
             *
             * @param msg gz IMU message
             */
            void on_imu_msg(const gz::msgs::IMU& msg) {
                ESP_LOGI(TAG, "on_imu_msg");
            }


        private:
            gz::transport::Node node_;
            const sitl::SimConfig sitl_cfg_;

    };

    std::unique_ptr<IMU> make_imu() {
        return std::make_unique<imu::IMU_SITL>();

    }

}
