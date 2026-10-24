#include <gtest/gtest.h>

#include <string>

#include "payload_controller/control_math.hpp"

namespace cm = payload::control_math;

TEST(ControlMathTest, WheelSetpointsStraight)
{
  const auto setpoints = cm::compute_wheel_setpoints(1.0, 0.0, 0.12132, 0.01611839, 500.0);

  EXPECT_NEAR(setpoints.left_mps, 1.0, 1e-9);
  EXPECT_NEAR(setpoints.right_mps, 1.0, 1e-9);
  EXPECT_NEAR(setpoints.left_rpm, setpoints.right_rpm, 1e-9);
}

TEST(ControlMathTest, WheelSetpointsTurning)
{
  const auto setpoints = cm::compute_wheel_setpoints(0.5, 2.0, 0.12132, 0.01611839, 500.0);

  EXPECT_LT(setpoints.left_mps, setpoints.right_mps);
  EXPECT_LT(setpoints.left_rpm, setpoints.right_rpm);
}

TEST(ControlMathTest, RpmFromCounts)
{
  // Half a revolution over 0.5s -> 60 RPM.
  const double rpm = cm::rpm_from_count_delta(308, 616, 0.5);
  EXPECT_NEAR(rpm, 60.0, 0.5);
}

TEST(ControlMathTest, PidOutputSaturatesAndIntegralClamps)
{
  cm::PidConfig cfg;
  cfg.kp = 0.1;
  cfg.ki = 1.0;
  cfg.kd = 0.0;
  cfg.i_clamp = 0.2;
  cfg.output_limit_norm = 0.5;
  cfg.stop_deadband_rpm = 0.0;

  cm::PidState state;

  for (int i = 0; i < 20; ++i) {
    (void)cm::pid_step(10.0, 0.0, 0.1, cfg, state);
  }

  const auto terms = cm::pid_step(10.0, 0.0, 0.1, cfg, state);
  EXPECT_NEAR(terms.i, 0.2, 1e-6);
  EXPECT_NEAR(terms.output, 0.5, 1e-6);
}

TEST(ControlMathTest, DeadbandResetsIntegralAndZeroesOutput)
{
  cm::PidConfig cfg;
  cfg.kp = 1.0;
  cfg.ki = 1.0;
  cfg.kd = 0.0;
  cfg.i_clamp = 5.0;
  cfg.output_limit_norm = 1.0;
  cfg.stop_deadband_rpm = 1.0;

  cm::PidState state;
  (void)cm::pid_step(5.0, 0.0, 0.1, cfg, state);
  ASSERT_GT(state.integral, 0.0);

  const auto terms = cm::pid_step(0.2, 0.0, 0.1, cfg, state);
  EXPECT_NEAR(state.integral, 0.0, 1e-9);
  EXPECT_NEAR(terms.output, 0.0, 1e-9);
}

TEST(ControlMathTest, ZieglerNicholsClassic)
{
  cm::PidGains gains;
  std::string error;

  ASSERT_TRUE(cm::compute_ziegler_nichols_classic(4.0, 2.0, gains, error));
  EXPECT_TRUE(error.empty());
  EXPECT_NEAR(gains.kp, 2.4, 1e-9);
  EXPECT_NEAR(gains.ki, 2.4, 1e-9);
  EXPECT_NEAR(gains.kd, 0.6, 1e-9);
}

TEST(ControlMathTest, ZieglerNicholsRejectsInvalidInputs)
{
  cm::PidGains gains;
  std::string error;

  EXPECT_FALSE(cm::compute_ziegler_nichols_classic(0.0, 1.0, gains, error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(cm::compute_ziegler_nichols_classic(1.0, 0.0, gains, error));
  EXPECT_FALSE(error.empty());
}
