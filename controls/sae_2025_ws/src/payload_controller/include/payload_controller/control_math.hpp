#ifndef CONTROL_MATH_HPP
#define CONTROL_MATH_HPP

#include <cstdint>
#include <string>

namespace payload::control_math {

struct WheelSetpoints {
    double left_mps {0.0};
    double right_mps {0.0};
    double left_rpm {0.0};
    double right_rpm {0.0};
};

struct PidConfig {
    double kp {0.0};
    double ki {0.0};
    double kd {0.0};
    double i_clamp {0.0};
    double output_limit_norm {1.0};
    double stop_deadband_rpm {0.0};
};

struct PidState {
    double integral {0.0};
    double prev_measured {0.0};
};

struct PidTerms {
    double error {0.0};
    double p {0.0};
    double i {0.0};
    double d {0.0};
    double output {0.0};
};

struct PidGains {
    double kp {0.0};
    double ki {0.0};
    double kd {0.0};
};

WheelSetpoints compute_wheel_setpoints(
    double linear_mps,
    double angular_rad_s,
    double wheel_separation_m,
    double wheel_radius_m,
    double max_wheel_rpm);

double rpm_from_count_delta(
    int64_t delta_counts,
    int output_cpr,
    double dt_s);

double low_pass_filter(double current, double previous_filtered, double alpha);

PidTerms pid_step(
    double setpoint_rpm,
    double measured_rpm,
    double dt_s,
    const PidConfig& config,
    PidState& state);

bool compute_ziegler_nichols_classic(
    double ku,
    double pu,
    PidGains& gains,
    std::string& error_message);

}  // namespace payload::control_math

#endif
