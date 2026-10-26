#include "payload_controller/control_math.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}

namespace payload::control_math
{

WheelSetpoints compute_wheel_setpoints(
    double linear_mps,
    double angular_rad_s,
    double wheel_separation_m,
    double wheel_radius_m,
    double max_wheel_rpm
)
{
  WheelSetpoints setpoints;

  setpoints.left_mps = linear_mps - 0.5 * angular_rad_s * wheel_separation_m;
  setpoints.right_mps = linear_mps + 0.5 * angular_rad_s * wheel_separation_m;

  const double mps_to_rpm = 60.0 / (kTwoPi * wheel_radius_m);
  setpoints.left_rpm = std::clamp(setpoints.left_mps * mps_to_rpm, -max_wheel_rpm, max_wheel_rpm);
  setpoints.right_rpm = std::clamp(setpoints.right_mps * mps_to_rpm, -max_wheel_rpm, max_wheel_rpm);

  return setpoints;
}

double rpm_from_count_delta(int64_t delta_counts, int output_cpr, double dt_s)
{
  if (output_cpr <= 0 || dt_s <= 0.0) {
    return 0.0;
  }

  return (static_cast<double>(delta_counts) / static_cast<double>(output_cpr)) * (60.0 / dt_s);
}

double low_pass_filter(double current, double previous_filtered, double alpha)
{
  const double a = std::clamp(alpha, 0.0, 1.0);
  return (a * current) + ((1.0 - a) * previous_filtered);
}

PidTerms pid_step(double setpoint_rpm, double measured_rpm, double dt_s, const PidConfig& config, PidState& state)
{
  PidTerms terms;
  terms.error = setpoint_rpm - measured_rpm;

  if (dt_s <= 0.0) {
    state.prev_measured = measured_rpm;
    return terms;
  }

  if (std::abs(setpoint_rpm) < config.stop_deadband_rpm) {
    state.integral = 0.0;
    state.prev_measured = measured_rpm;
    return terms;
  }

  terms.p = config.kp * terms.error;

  state.integral += config.ki * terms.error * dt_s;
  state.integral = std::clamp(state.integral, -config.i_clamp, config.i_clamp);
  terms.i = state.integral;

  const double measured_derivative = (measured_rpm - state.prev_measured) / dt_s;
  terms.d = -config.kd * measured_derivative;

  const double unclamped = terms.p + terms.i + terms.d;
  terms.output = std::clamp(unclamped, -config.output_limit_norm, config.output_limit_norm);

  state.prev_measured = measured_rpm;
  return terms;
}

bool compute_ziegler_nichols_classic(double ku, double pu, PidGains& gains, std::string& error_message)
{
  if (ku <= 0.0) {
    error_message = "Ku must be > 0";
    return false;
  }
  if (pu <= 0.0) {
    error_message = "Pu must be > 0";
    return false;
  }

  gains.kp = 0.6 * ku;
  gains.ki = 1.2 * ku / pu;
  gains.kd = 0.075 * ku * pu;
  error_message.clear();
  return true;
}

}  // namespace payload::control_math
