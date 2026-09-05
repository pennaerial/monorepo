#include <pigpiod_if2.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "payload_controller/encoder.hpp"
#include "payload_controller/motor.hpp"

// ---- Pins from payload_params.yaml ----
// static constexpr int SN_LEFT_PWM  = 18;
// static constexpr int SN_LEFT_IN1  = 15;
// static constexpr int SN_LEFT_IN2  = 14;
// static constexpr int SN_RIGHT_PWM = 13;
// static constexpr int SN_RIGHT_IN1 = 20;
// static constexpr int SN_RIGHT_IN2 = 16;

static constexpr int SN_RIGHT_PWM = 18;
static constexpr int SN_RIGHT_IN1 = 15;
static constexpr int SN_RIGHT_IN2 = 14;
static constexpr int SN_LEFT_PWM = 13;
static constexpr int SN_LEFT_IN1 = 20;
static constexpr int SN_LEFT_IN2 = 16;

static constexpr int ENCA1 = 5;
static constexpr int ENCA2 = 6;
static constexpr int ENCB1 = 10;
static constexpr int ENCB2 = 9;

constexpr float FREQ = 200.0f;
static constexpr int ENC_CPR = 2400;

static int g_pi = -1;
static void pause(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static void pause_print(int ms, QuadratureEncoder& enc_a, QuadratureEncoder& enc_b)
{
  auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  while (std::chrono::steady_clock::now() < end) {
    printf(
        "  enc_a: %lld (%.1f deg)   enc_b: %lld (%.1f deg)\n", static_cast<long long>(enc_a.count()), enc_a.angle_deg(),
        static_cast<long long>(enc_b.count()), enc_b.angle_deg()
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

static Motor* g_motor_a = nullptr;
static Motor* g_motor_b = nullptr;

static void on_sigint(int)
{
  printf("\nCaught SIGINT — coasting motors and cleaning up\n");
  if (g_motor_a) {
    g_motor_a->coast();
  }
  if (g_motor_b) {
    g_motor_b->coast();
  }
  if (g_pi >= 0) {
    pigpio_stop(g_pi);
  }
  rclcpp::shutdown();
  std::exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  g_pi = pigpio_start(nullptr, nullptr);
  if (g_pi < 0) {
    printf("ERROR: pigpio_start() failed (err=%d). Is pigpiod running?\n", g_pi);
    rclcpp::shutdown();
    return 1;
  }
  printf("Connected to pigpiod\n");

  std::unique_ptr<Motor> motor_right;
  std::unique_ptr<Motor> motor_left;
  printf("Running SNMotor test\n");
  motor_right = std::make_unique<SNMotor>(
      g_pi, SN_RIGHT_PWM, SN_RIGHT_IN1, SN_RIGHT_IN2, static_cast<int>(FREQ), MotorType::RIGHT
  );
  motor_left =
      std::make_unique<SNMotor>(g_pi, SN_LEFT_PWM, SN_LEFT_IN1, SN_LEFT_IN2, static_cast<int>(FREQ), MotorType::LEFT);
  g_motor_a = motor_right.get();
  g_motor_b = motor_left.get();
  std::signal(SIGINT, on_sigint);

  QuadratureEncoder enc_a(g_pi, ENCA1, ENCA2, ENC_CPR, MotorType::RIGHT);
  QuadratureEncoder enc_b(g_pi, ENCB1, ENCB2, ENC_CPR, MotorType::LEFT);

  std::cout << "RIGHT MOTOR FORWARD 70%" << std::endl;
  motor_right->forward(70.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "RIGHT MOTOR FORWARD 20%" << std::endl;
  motor_right->forward(20.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "RIGHT MOTOR REVERSE 100%" << std::endl;
  motor_right->reverse(100.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "RIGHT MOTOR REVERSE 50%" << std::endl;
  motor_right->reverse(50.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "RIGHT MOTOR STOPPING:" << std::endl;
  motor_right->coast();
  pause_print(1000, enc_a, enc_b);

  std::cout << "LEFT MOTOR FORWARD 70%" << std::endl;
  motor_left->forward(70.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "LEFT MOTOR FORWARD 20%" << std::endl;
  motor_left->forward(20.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "LEFT MOTOR REVERSE 100%" << std::endl;
  motor_left->reverse(100.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "LEFT MOTOR REVERSE 50%" << std::endl;
  motor_left->reverse(50.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "Stopping motors" << std::endl;
  motor_right->coast();
  motor_left->coast();
  pause_print(1000, enc_a, enc_b);

  std::cout << "BOTH FORWARD" << std::endl;
  motor_right->forward(100.0f);
  motor_left->forward(100.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "BOTH REVERSE" << std::endl;
  motor_right->reverse(100.0f);
  motor_left->reverse(100.0f);
  pause_print(1000, enc_a, enc_b);

  std::cout << "Stopping motors" << std::endl;
  motor_right->coast();
  motor_left->coast();

  printf("\n=== set_speed tests ===\n");

  printf("RIGHT set_speed(+0.7)\n");
  motor_right->set_speed(0.7f);
  pause_print(2000, enc_a, enc_b);

  printf("RIGHT set_speed(-1.0)\n");
  motor_right->set_speed(-1.0f);
  pause_print(2000, enc_a, enc_b);

  printf("RIGHT set_speed(0.0)\n");
  motor_right->set_speed(0.0f);
  pause_print(2000, enc_a, enc_b);

  printf("LEFT set_speed(+0.7)\n");
  motor_left->set_speed(0.7f);
  pause_print(2000, enc_a, enc_b);

  printf("LEFT set_speed(-1.0)\n");
  motor_left->set_speed(-1.0f);
  pause_print(2000, enc_a, enc_b);

  printf("LEFT set_speed(0.0)\n");
  motor_left->set_speed(0.0f);
  pause_print(2000, enc_a, enc_b);

  printf("BOTH set_speed(+1.0)\n");
  motor_right->set_speed(1.0f);
  motor_left->set_speed(1.0f);
  pause_print(2000, enc_a, enc_b);

  printf("BOTH set_speed(-1.0)\n");
  motor_right->set_speed(-1.0f);
  motor_left->set_speed(-1.0f);
  pause_print(2000, enc_a, enc_b);

  motor_right->set_speed(0.0f);
  motor_left->set_speed(0.0f);
  pause(500);

  printf("TURN LEFT: right +0.7, left -0.7\n");
  motor_right->set_speed(0.7f);
  motor_left->set_speed(-0.7f);
  pause_print(3000, enc_a, enc_b);

  motor_right->set_speed(0.0f);
  motor_left->set_speed(0.0f);
  pause(500);

  printf("TURN RIGHT: right -0.7, left +0.7\n");
  motor_right->set_speed(-0.7f);
  motor_left->set_speed(0.7f);
  pause_print(3000, enc_a, enc_b);

  motor_right->set_speed(0.0f);
  motor_left->set_speed(0.0f);

  printf("\nDone.\n");
  pause(500);
  pigpio_stop(g_pi);
  rclcpp::shutdown();
  return 0;
}
