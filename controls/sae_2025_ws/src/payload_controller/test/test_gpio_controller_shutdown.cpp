#include <gtest/gtest.h>

#include <memory>

#include "payload_controller/gpio_controller.hpp"

namespace
{

class RecordingMotor : public Motor
{
public:
  void set_speed(float speed) override
  {
    last_speed = speed;
  }

  void forward(float duty) override
  {
    last_forward = duty;
  }

  void reverse(float duty) override
  {
    last_reverse = duty;
  }

  void coast() override
  {
    ++coast_calls;
  }

  void hard_brake() override
  {
    ++hard_brake_calls;
  }

  float last_speed{0.0f};
  float last_forward{0.0f};
  float last_reverse{0.0f};
  int coast_calls{0};
  int hard_brake_calls{0};
};

TEST(GPIOControllerShutdownTest, SafeShutdownBrakesDriveMotors)
{
  auto left_motor = std::make_unique<RecordingMotor>();
  auto right_motor = std::make_unique<RecordingMotor>();
  auto* left = left_motor.get();
  auto* right = right_motor.get();

  GPIOController controller(std::move(left_motor), std::move(right_motor));

  controller.safe_shutdown();
  controller.safe_shutdown();

  EXPECT_EQ(left->hard_brake_calls, 1);
  EXPECT_EQ(right->hard_brake_calls, 1);
  EXPECT_EQ(left->coast_calls, 0);
  EXPECT_EQ(right->coast_calls, 0);
}

}  // namespace
