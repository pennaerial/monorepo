#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>

#include "imu.hpp"
#include "payload.hpp"
#include "sample_fifo.hpp"

namespace drivers
{
class TestIMU : public IMU
{
public:
  void start() override {}
};
IMU* IMU::instance()
{
  static TestIMU imu;
  return &imu;
}
class TestMotor : public Motor
{
public:
  void start() override {}
  void publish(double) override {}
};
Motor& Motor::instance(Side side)
{
  static TestMotor left, right;
  return side == Side::LEFT ? left : right;
}
void Camera::start() {}
}  // namespace drivers

sensor_msgs_msg_Imu make_sample(int64_t timestamp_ns, double acceleration_x)
{
  sensor_msgs_msg_Imu sample{};
  sample.header.stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000LL);
  sample.header.stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000LL);
  sample.linear_acceleration.x = acceleration_x;
  sample.linear_acceleration.z = 9.81;
  return sample;
}

int main()
{
  // The same bounded FIFO is used for Gazebo staging and the estimator's sample history.
  util::SampleFifo<int, 4> fifo;
  for (int i = 0; i < 6; ++i)
    fifo.push(i);
  assert(fifo.size() == 4 && fifo.take_dropped() == 2 && fifo.take_dropped() == 0);
  for (int expected = 2; expected < 6; ++expected) {
    int sample;
    assert(fifo.pop(sample) && sample == expected);
  }
  fifo.push(6);
  int next;
  assert(fifo.pop(next) && next == 6 && !fifo.pop(next));

  drivers::TestIMU sensor;
  for (int i = 0; i < 25; ++i)
    sensor.write_latest(make_sample(i * 2000000LL, i));
  uint32_t telemetry_cursor = 0;
  sensor_msgs_msg_Imu latest{};
  assert(sensor.get_latest_if_updated(telemetry_cursor, latest));
  assert(latest.linear_acceleration.x == 24);
  std::array<drivers::IMU::Sample, drivers::IMU::SAMPLE_BUFFER_CAPACITY> samples;
  std::size_t dropped;
  assert(sensor.read_samples(samples.data(), samples.size(), dropped) == 25 && dropped == 0);
  for (int i = 0; i < 25; ++i) {
    assert(samples[i].timestamp_ns == i * 2000000LL && samples[i].linear_acceleration[0] == i);
  }
  assert(sensor.read_samples(samples.data(), samples.size(), dropped) == 0 && dropped == 0);
  assert(sensor.get_latest().linear_acceleration.x == 24);  // Draining history preserves telemetry.
  for (int i = 0; i < 80; ++i)
    sensor.write_latest(make_sample(i * 2000000LL, i));
  assert(sensor.read_samples(samples.data(), samples.size(), dropped) == 64 && dropped == 16);
  for (int i = 0; i < 64; ++i)
    assert(samples[i].linear_acceleration[0] == i + 16);
  std::puts("PASS: FIFO order, wraparound, overflow counts, and independent telemetry reads");

  auto* payload = Payload::instance();
  payload->init(Eigen::Vector3f::Zero());
  for (int i = 0; i < 25; ++i)
    drivers::IMU::instance()->write_latest(make_sample(i * 2000000LL, 1.0));
  payload->publish_sensor_debug();  // Telemetry must not consume any EKF measurements.
  payload->update_estimator();
  assert(std::abs(EKF::instance().get_position().x() - 0.5f * 0.048f * 0.048f) < 1e-7f);
  for (int i = 25; i < 50; ++i)
    drivers::IMU::instance()->write_latest(make_sample(i * 2000000LL, 1.0));
  payload->update_estimator();
  const float position = EKF::instance().get_position().x();
  assert(std::abs(position - 0.5f * 0.098f * 0.098f) < 1e-7f);
  payload->update_estimator();
  assert(EKF::instance().get_position().x() == position);  // No replay without fresh data.
  std::puts("PASS: real payload/EKF integrate every 500 Hz sample across multiple batches");

  EKF::instance().init(Eigen::Vector3f::Zero());
  for (int i = 0; i < 6; ++i) {
    drivers::IMU::instance()->write_latest(make_sample(i * 2000000LL, i == 1 ? 10.0 : 0.0));
  }
  payload->update_estimator();
  assert(std::abs(EKF::instance().get_position().x() - 0.00018f) < 1e-8f);
  std::puts("PASS: a short acceleration pulse between task wakes reaches the EKF");
}
