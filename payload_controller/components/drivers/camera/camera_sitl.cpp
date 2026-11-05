#include <gz/msgs/pose_v.pb.h>

#include <cmath>
#include <gz/transport/Node.hh>
#include <random>
#include <string>

#include "camera.hpp"
#include "esp_log.h"
#include "sitl_runtime.hpp"

namespace
{
// Provisional camera model: 10 Hz, 5 cm per-axis white noise and a fixed
// random per-axis bias (2 cm SD). Fixed seed makes Gazebo tests reproducible.
constexpr int64_t SAMPLE_PERIOD_NS = 100000000;
constexpr float NOISE_SD_M = 0.05f;  // Matches EKF camera measurement noise.
constexpr float BIAS_SD_M = 0.02f;

class SimCamera
{
public:
  SimCamera()
  {
    std::normal_distribution<double> bias(0.0, BIAS_SD_M);
    bias_ = {bias(random_), bias(random_), bias(random_)};
    const auto& config = sitl::get_config();
    model_ = config.gz_model;
    // SceneBroadcaster publishes model poses in world coordinates here.
    const std::string topic = std::string("/world/") + config.gz_world + "/pose/info";
    if (!node_.Subscribe(topic, &SimCamera::on_pose, this)) {
      ESP_LOGE("CAMERA_SITL", "Failed to subscribe to %s", topic.c_str());
    }
  }

private:
  void on_pose(const gz::msgs::Pose_V& message)
  {
    if (!message.has_header() || !message.header().has_stamp()) {
      return;
    }
    const auto& stamp = message.header().stamp();
    const int64_t now_ns = static_cast<int64_t>(stamp.sec()) * 1000000000LL + stamp.nsec();
    if (has_sample_ && now_ns >= last_sample_ns_ && now_ns - last_sample_ns_ < SAMPLE_PERIOD_NS) {
      return;
    }
    for (const auto& pose : message.pose()) {
      if (pose.name() != model_ || !pose.has_position()) {
        continue;
      }
      const auto& p = pose.position();
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
        return;
      }
      // The current model origin and IMU origin coincide; lever arm is zero.
      geometry_msgs_msg_Vector3 position{
          p.x() + bias_.x + noise_(random_), p.y() + bias_.y + noise_(random_), p.z() + bias_.z + noise_(random_)
      };
      drivers::Camera::instance().write_latest(position);
      has_sample_ = true;
      last_sample_ns_ = now_ns;
      return;
    }
  }

  std::string model_;
  std::mt19937 random_{42};
  std::normal_distribution<double> noise_{0.0, NOISE_SD_M};
  geometry_msgs_msg_Vector3 bias_{};
  bool has_sample_ = false;
  int64_t last_sample_ns_ = 0;
  // Declared last so subscriptions stop before callback state is destroyed.
  gz::transport::Node node_;
};
}  // namespace

void drivers::Camera::start()
{
  static SimCamera camera;
}
