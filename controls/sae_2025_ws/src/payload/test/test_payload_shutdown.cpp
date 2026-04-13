#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "payload/controller.hpp"
#include "payload/payload.hpp"

namespace
{

class RecordingController : public Controller
{
public:
  void initialize(rclcpp::Node * node) override
  {
    (void)node;
    ++initialize_calls;
  }

  void drive_command(double linear, double angular) override
  {
    drive_commands.emplace_back(linear, angular);
  }

  void servo_command(double degree) override
  {
    servo_commands.push_back(degree);
  }

  void safe_shutdown() override
  {
    ++safe_shutdown_calls;
    drive_commands.emplace_back(0.0, 0.0);
    servo_commands.push_back(0.0);
  }

  int initialize_calls {0};
  int safe_shutdown_calls {0};
  std::vector<std::pair<double, double>> drive_commands;
  std::vector<double> servo_commands;
};

class PayloadShutdownTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(PayloadShutdownTest, DestructorInvokesControllerSafeShutdownOnce) {
  auto controller = std::make_shared<RecordingController>();

  {
    auto payload = std::make_unique<Payload>("payload_shutdown_test", controller);
    payload->init();
  }

  EXPECT_EQ(controller->initialize_calls, 1);
  EXPECT_EQ(controller->safe_shutdown_calls, 1);
  ASSERT_FALSE(controller->drive_commands.empty());
  EXPECT_DOUBLE_EQ(controller->drive_commands.back().first, 0.0);
  EXPECT_DOUBLE_EQ(controller->drive_commands.back().second, 0.0);
  ASSERT_FALSE(controller->servo_commands.empty());
  EXPECT_DOUBLE_EQ(controller->servo_commands.back(), 0.0);
}

TEST_F(PayloadShutdownTest, PrepareForShutdownIsIdempotent) {
  auto controller = std::make_shared<RecordingController>();

  {
    auto payload = std::make_unique<Payload>("payload_shutdown_test", controller);
    payload->init();
    payload->prepare_for_shutdown();
    payload->prepare_for_shutdown();
  }

  EXPECT_EQ(controller->initialize_calls, 1);
  EXPECT_EQ(controller->safe_shutdown_calls, 1);
  ASSERT_FALSE(controller->drive_commands.empty());
  EXPECT_DOUBLE_EQ(controller->drive_commands.back().first, 0.0);
  EXPECT_DOUBLE_EQ(controller->drive_commands.back().second, 0.0);
  ASSERT_FALSE(controller->servo_commands.empty());
  EXPECT_DOUBLE_EQ(controller->servo_commands.back(), 0.0);
}

}  // namespace
