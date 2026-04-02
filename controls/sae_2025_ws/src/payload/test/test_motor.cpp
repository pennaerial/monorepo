#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "payload/motor.hpp"
#include "payload/servo.hpp"
#include "payload/encoder.hpp"

// ---- Pins from payload_params.yaml ----
// SN754410 pins
//
//
// LEFT_PWM: 13
// LEFT_IN1: 20
// LEFT_IN2: 16
// RIGHT_PWM: 18
// RIGHT_IN1: 15
// RIGHT_IN2: 14
//
//
static constexpr int SN_LEFT_PWM = 13;
static constexpr int SN_LEFT_IN1  = 20;
static constexpr int SN_LEFT_IN2  = 16;
static constexpr int SN_RIGHT_PWM = 18;
static constexpr int SN_RIGHT_IN1  = 15;
static constexpr int SN_RIGHT_IN2  = 14;



//ENCODER PINS
// ENCA_CH1: 6
// ENCA_CH2: 5
// ENCB_CH1: 10
// ENCB_CH2: 9

static constexpr int ENCA1  = 6;
static constexpr int ENCA2  = 5;
static constexpr int ENCB1  = 10;
static constexpr int ENCB2  = 9;


constexpr int SERVO = 14;
constexpr float FREQ = 200.0; //200 Hz

static constexpr int ENC_CPR = 2400; // counts per rev after 4x decode

static void pause(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static void pause_print(int ms, QuadratureEncoder& enc_a, QuadratureEncoder& enc_b)
{
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
        printf("  enc_a: %lld (%.1f deg)   enc_b: %lld (%.1f deg)\n",
               static_cast<long long>(enc_a.count()), enc_a.angle_deg(),
               static_cast<long long>(enc_b.count()), enc_b.angle_deg());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ---- Signal handling ----
static int       g_handle  = -1;
static Motor*    g_motor_a = nullptr;
static Motor*    g_motor_b = nullptr;

static void on_sigint(int)
{
    printf("\nCaught SIGINT — coasting motors and cleaning up\n");
    if (g_motor_a) g_motor_a->coast();
    if (g_motor_b) g_motor_b->coast();
    if (g_handle >= 0) lgGpiochipClose(g_handle);
    rclcpp::shutdown();
    std::exit(0);
}

// ---- Test sequences ----

// ---- Main ----
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Open gpiochip — try RPi 5 (chip4) then RPi 4 (chip0)
    int h = -1;
    for (int chip : {4, 0}) {
        h = lgGpiochipOpen(chip);
        if (h >= 0) { printf("Opened gpiochip%d (handle=%d)\n", chip, h); break; }
        printf("gpiochip%d failed: %d\n", chip, h);
    }
    if (h < 0) {
        printf("ERROR: Could not open any gpiochip\n");
        rclcpp::shutdown();
        return 1;
    }
    g_handle = h;

    std::unique_ptr<Motor> motor_right;
    std::unique_ptr<Motor> motor_left;
    printf("Running SNMotor test\n");
    motor_left = std::make_unique<SNMotor>(h, SN_RIGHT_PWM, SN_RIGHT_IN1, SN_RIGHT_IN2, FREQ, MotorType::RIGHT);
    motor_right = std::make_unique<SNMotor>(h, SN_LEFT_PWM, SN_LEFT_IN1, SN_LEFT_IN2, FREQ, MotorType::LEFT);
    g_motor_a = motor_right.get();
    g_motor_b = motor_left.get();
    std::signal(SIGINT, on_sigint);

    QuadratureEncoder enc_a(h, ENCA1, ENCA2, ENC_CPR, MotorType::RIGHT);
    QuadratureEncoder enc_b(h, ENCB1, ENCB2, ENC_CPR, MotorType::LEFT);

    std::cout << "RIGHT MOTOR FORWARD 70%" << std::endl;
    motor_right->forward(70.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "RIGHT MOTOR FORWARD 20%" << std::endl;
    motor_right->forward(20.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "RIGHT MOTOR REVERSE 100%" << std::endl;
    motor_right->reverse(100.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "RIGHT MOTOR REVERSE 50%" << std::endl;
    motor_right->reverse(50.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "RIGHT MOTOR STOPPING:" << std::endl;
    motor_right->coast();
    pause_print(2000, enc_a, enc_b);

    std::cout << "LEFT MOTOR FORWARD 70%" << std::endl;
    motor_left->forward(70.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "LEFT MOTOR FORWARD 20%" << std::endl;
    motor_left->forward(20.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "LEFT MOTOR REVERSE 100%" << std::endl;
    motor_left->reverse(100.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "LEFT MOTOR B REVERSE 50%" << std::endl;
    motor_left->reverse(50.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "Stopping motors" << std::endl;
    motor_right->coast();
    motor_left->coast();
    pause_print(2000, enc_a, enc_b);

    std::cout << "BOTH FORWARD" << std::endl;
    motor_right->forward(100.0f);
    motor_left->forward(100.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "BOTH REVERSE" << std::endl;
    motor_right->reverse(100.0f);
    motor_left->reverse(100.0f);
    pause_print(2000, enc_a, enc_b);

    std::cout << "Stopping motors" << std::endl;
    motor_right->coast();
    motor_left->coast();
    printf("\nDone.\n");

    pause(500);
    lgGpiochipClose(h);
    rclcpp::shutdown();
    return 0;
}
