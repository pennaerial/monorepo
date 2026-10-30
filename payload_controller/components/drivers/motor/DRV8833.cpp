#include "DRV8833.hpp"

#include <algorithm>
#include <cmath>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "DRV8833";

namespace
{

// All four inputs share one PWM timer. 20 kHz is above hearing range (no motor whine) and well
// within what the DRV8833 inputs can switch at.
const ledc_mode_t PWM_SPEED_MODE = LEDC_LOW_SPEED_MODE;  // The only mode on ESP32-S3 / C3
const ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
const ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT;
const uint32_t PWM_FREQUENCY_HZ = 20000;
// LEDC duty runs from 0 to 2^resolution inclusive, where 2^resolution holds the pin high
const uint32_t PWM_FULL_DUTY = 1u << PWM_RESOLUTION;

// Wheel speed at 100% duty. Used to turn rad/s commands into duty cycles.
// TODO: Placeholder value, measure on the real payload (run at full duty and read the encoders) and update
const double MAX_WHEEL_SPEED_RAD_S = 30.0;

/// The ESP pins and PWM channels wired to one of the DRV8833's two H-bridges.
struct BridgeConfig {
  gpio_num_t in1_pin;
  gpio_num_t in2_pin;
  ledc_channel_t in1_channel;
  ledc_channel_t in2_channel;
  // Flips forward / reverse if a motor turns the wrong way for a positive command
  bool inverted;
};

// Payload v2 wiring: AIN1 = 6, AIN2 = 7, BIN1 = 10, BIN2 = 8
// TODO: Confirm these pins on the real board, and that the board is an ESP32-S3. The v2 diagram's Tx 21 / Rx 20
// labels match an ESP32-C3, which would need a different build target.
// TODO: Set inverted per side once we can check which way each wheel spins for a positive command
const BridgeConfig LEFT_BRIDGE = {GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_0, LEDC_CHANNEL_1, false};
const BridgeConfig RIGHT_BRIDGE = {GPIO_NUM_10, GPIO_NUM_8, LEDC_CHANNEL_2, LEDC_CHANNEL_3, false};

const BridgeConfig& bridge_for(Side side)
{
  return side == Side::LEFT ? LEFT_BRIDGE : RIGHT_BRIDGE;
}

const char* side_name(Side side)
{
  return side == Side::LEFT ? "left" : "right";
}

void configure_pwm_channel(gpio_num_t pin, ledc_channel_t channel)
{
  ledc_channel_config_t channel_cfg = {};
  channel_cfg.gpio_num = pin;
  channel_cfg.speed_mode = PWM_SPEED_MODE;
  channel_cfg.channel = channel;
  channel_cfg.timer_sel = PWM_TIMER;
  channel_cfg.duty = 0;  // Both inputs low = coast, so the motor stays still until commanded
  channel_cfg.hpoint = 0;

  ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

}  // namespace

namespace drivers
{

DRV8833::DRV8833(Side side) : side_(side) {}

void DRV8833::start()
{
  // Both bridges run off the same timer, so it only needs to be set up by whichever side starts first
  static bool timer_configured = false;
  if (!timer_configured) {
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = PWM_SPEED_MODE;
    timer_cfg.duty_resolution = PWM_RESOLUTION;
    timer_cfg.timer_num = PWM_TIMER;
    timer_cfg.freq_hz = PWM_FREQUENCY_HZ;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    timer_configured = true;
  }

  // nSLEEP and nFAULT are left unconnected, so this relies on the breakout pulling nSLEEP high
  // TODO: Check the breakout's schematic. nSLEEP has an internal pulldown, so if the breakout doesn't pull it
  // high the motors never move. Wiring it to a GPIO would also let us sleep the driver to save battery.
  const BridgeConfig& bridge = bridge_for(side_);
  configure_pwm_channel(bridge.in1_pin, bridge.in1_channel);
  configure_pwm_channel(bridge.in2_pin, bridge.in2_channel);
  started_ = true;

  ESP_LOGI(TAG, "Started %s motor on IN1: GPIO %i, IN2: GPIO %i", side_name(side_), bridge.in1_pin, bridge.in2_pin);
}

void DRV8833::publish(double rad_s)
{
  // Writing to channels that haven't been configured would trip ESP_ERROR_CHECK
  if (!started_) {
    return;
  }

  if (!std::isfinite(rad_s)) {
    rad_s = 0.0;
  }

  const BridgeConfig& bridge = bridge_for(side_);
  const bool reverse = (rad_s < 0.0) != bridge.inverted;
  const double drive_fraction = std::min(std::abs(rad_s) / MAX_WHEEL_SPEED_RAD_S, 1.0);

  // Slow decay mode (datasheet Table 2): hold one input high and PWM the other. While the PWM input
  // is low the motor is driven, while it is high both inputs are high and the motor brakes. So the
  // PWM input's duty is the inverse of how hard we drive, and a 0 command brakes the motor.
  const uint32_t pwm_duty = static_cast<uint32_t>(std::lround((1.0 - drive_fraction) * PWM_FULL_DUTY));

  if (reverse) {
    write_inputs(pwm_duty, PWM_FULL_DUTY);  // xIN1 = PWM, xIN2 = 1
  } else {
    write_inputs(PWM_FULL_DUTY, pwm_duty);  // xIN1 = 1, xIN2 = PWM
  }
}

void DRV8833::write_inputs(uint32_t in1_duty, uint32_t in2_duty)
{
  const BridgeConfig& bridge = bridge_for(side_);

  ESP_ERROR_CHECK(ledc_set_duty(PWM_SPEED_MODE, bridge.in1_channel, in1_duty));
  ESP_ERROR_CHECK(ledc_update_duty(PWM_SPEED_MODE, bridge.in1_channel));
  ESP_ERROR_CHECK(ledc_set_duty(PWM_SPEED_MODE, bridge.in2_channel, in2_duty));
  ESP_ERROR_CHECK(ledc_update_duty(PWM_SPEED_MODE, bridge.in2_channel));
}

Motor& Motor::instance(Side side)
{
  static DRV8833 left(Side::LEFT);
  static DRV8833 right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}

}  // namespace drivers
