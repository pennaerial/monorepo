#include <pigpiod_if2.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "payload_controller/servo.hpp"


constexpr int SERVO_PIN = 23;
constexpr float PULSE_MIN = 600.0f;  // used to be 700.0f, but that was too low for the servo to reliably reach 0
                                     // degrees. Adjusted after testing with an oscilloscope.
constexpr float PULSE_MAX = 2200.0f;
constexpr int FREQ = 50;

static int g_pi = -1;
static void pause(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static void on_sigint(int)
{
  std::printf("\nCaught SIGINT -- closing servo and cleaning up\n");
  if (g_pi >= 0) {
    pigpio_stop(g_pi);
  }
  rclcpp::shutdown();
  std::exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, on_sigint);

  g_pi = pigpio_start(nullptr, nullptr);
  if (g_pi < 0) {
    std::printf("ERROR: pigpio_start() failed (err=%d). Is pigpiod running?\n", g_pi);
    rclcpp::shutdown();
    return 1;
  }
  std::printf("Connected to pigpiod\n");

  Servo servo(g_pi, SERVO_PIN, FREQ, PULSE_MIN, PULSE_MAX);

  std::cout << "NORMALIZED 0.0 (pulse_min=" << PULSE_MIN << "us)" << std::endl;
  servo.normalized_setpoint(0.0f);
  pause(2000);

  std::cout << "NORMALIZED 0.5 (midpoint)" << std::endl;
  servo.normalized_setpoint(0.5f);
  pause(2000);

  std::cout << "NORMALIZED 1.0 (pulse_max=" << PULSE_MAX << "us)" << std::endl;
  servo.normalized_setpoint(1.0f);
  pause(2000);

  std::cout << "CLOSE (0 deg) -- hook clamped shut" << std::endl;
  servo.degree_setpoint(0.0f);
  pause(2000);

  std::cout << "OPEN (180 deg) -- hook arm extended" << std::endl;
  servo.degree_setpoint(180.0f);
  pause(2000);

  std::cout << "CLOSE (0 deg) -- clamping shut" << std::endl;
  servo.degree_setpoint(0.0f);
  pause(2000);

  pause(500);
  pigpio_stop(g_pi);
  rclcpp::shutdown();
  return 0;
}
