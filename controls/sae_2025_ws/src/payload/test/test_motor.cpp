#include <rclcpp/rclcpp.hpp>
#include <pigpio.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "payload/encoder.hpp"
#include "payload/motor.hpp"
// ---- Pins from payload_params.yaml ----
// SN754410 pins
static constexpr int SN_LEFT_PWM = 18;
static constexpr int SN_LEFT_IN1 = 15;
static constexpr int SN_LEFT_IN2 = 14;
static constexpr int SN_RIGHT_PWM = 13;
static constexpr int SN_RIGHT_IN1 = 20;
static constexpr int SN_RIGHT_IN2 = 16;


//ENCODER PINS
static constexpr int ENCA1  = 6;
static constexpr int ENCA2  = 5;
static constexpr int ENCB1  = 10;
static constexpr int ENCB2  = 9;

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
static Motor*    g_motor_a = nullptr;
static Motor*    g_motor_b = nullptr;

static void on_sigint(int)
{
    printf("\nCaught SIGINT — coasting motors and cleaning up\n");
    if (g_motor_a) g_motor_a->coast();
    if (g_motor_b) g_motor_b->coast();
    gpioTerminate();
    rclcpp::shutdown();
    std::exit(0);
}

// ---- Test sequences ----

// ---- Main ----
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int rc = gpioInitialise();
    if (rc < 0) {
        printf("ERROR: gpioInitialise() failed (err=%d)\n", rc);
        rclcpp::shutdown();
        return 1;
    }
    printf("pigpio initialised (version=%d)\n", rc);

    std::unique_ptr<Motor> motor_right;
    std::unique_ptr<Motor> motor_left;
    printf("Running SNMotor test\n");
    motor_right = std::make_unique<SNMotor>(
        SN_RIGHT_PWM, SN_RIGHT_IN1, SN_RIGHT_IN2, FREQ, MotorType::RIGHT);
    motor_left = std::make_unique<SNMotor>(
        SN_LEFT_PWM, SN_LEFT_IN1, SN_LEFT_IN2, FREQ, MotorType::LEFT);
    g_motor_a = motor_right.get();
    g_motor_b = motor_left.get();
    std::signal(SIGINT, on_sigint);

    QuadratureEncoder enc_a(ENCA1, ENCA2, ENC_CPR, MotorType::RIGHT);
    QuadratureEncoder enc_b(ENCB1, ENCB2, ENC_CPR, MotorType::LEFT);

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

    printf("\n--- RIGHT motor set_speed ---\n");
    printf("RIGHT set_speed(+0.7) -- forward 70%%\n");
    motor_right->set_speed(0.7f);
    pause_print(2000, enc_a, enc_b);

    printf("RIGHT set_speed(+0.2) -- forward 20%%\n");
    motor_right->set_speed(0.2f);
    pause_print(2000, enc_a, enc_b);

    printf("RIGHT set_speed(-1.0) -- reverse 100%%\n");
    motor_right->set_speed(-1.0f);
    pause_print(2000, enc_a, enc_b);

    printf("RIGHT set_speed(-0.5) -- reverse 50%%\n");
    motor_right->set_speed(-0.5f);
    pause_print(2000, enc_a, enc_b);

    printf("RIGHT set_speed(0.0) -- coast\n");
    motor_right->set_speed(0.0f);
    pause_print(2000, enc_a, enc_b);

    printf("\n--- LEFT motor set_speed ---\n");
    printf("LEFT set_speed(+0.7) -- forward 70%%\n");
    motor_left->set_speed(0.7f);
    pause_print(2000, enc_a, enc_b);

    printf("LEFT set_speed(+0.2) -- forward 20%%\n");
    motor_left->set_speed(0.2f);
    pause_print(2000, enc_a, enc_b);

    printf("LEFT set_speed(-1.0) -- reverse 100%%\n");
    motor_left->set_speed(-1.0f);
    pause_print(2000, enc_a, enc_b);

    printf("LEFT set_speed(-0.5) -- reverse 50%%\n");
    motor_left->set_speed(-0.5f);
    pause_print(2000, enc_a, enc_b);

    printf("LEFT set_speed(0.0) -- coast\n");
    motor_left->set_speed(0.0f);
    pause_print(2000, enc_a, enc_b);

    printf("\n--- Both motors: straight drive via set_speed ---\n");
    printf("BOTH set_speed(+1.0) -- full forward\n");
    motor_right->set_speed(1.0f);
    motor_left->set_speed(1.0f);
    pause_print(2000, enc_a, enc_b);

    printf("BOTH set_speed(-1.0) -- full reverse\n");
    motor_right->set_speed(-1.0f);
    motor_left->set_speed(-1.0f);
    pause_print(2000, enc_a, enc_b);

    motor_right->set_speed(0.0f);
    motor_left->set_speed(0.0f);
    pause(500);

    printf("\n--- Turn simulation: opposite-sign set_speed (key test) ---\n");
    printf("TURN LEFT: right set_speed(+0.7), left set_speed(-0.7)\n");
    printf("  Expected: enc_a (right) positive, enc_b (left) negative\n");
    motor_right->set_speed(0.7f);
    motor_left->set_speed(-0.7f);
    pause_print(3000, enc_a, enc_b);

    motor_right->set_speed(0.0f);
    motor_left->set_speed(0.0f);
    pause(500);

    printf("TURN RIGHT: right set_speed(-0.7), left set_speed(+0.7)\n");
    printf("  Expected: enc_a (right) negative, enc_b (left) positive\n");
    motor_right->set_speed(-0.7f);
    motor_left->set_speed(0.7f);
    pause_print(3000, enc_a, enc_b);

    motor_right->set_speed(0.0f);
    motor_left->set_speed(0.0f);

    printf("\nDone.\n");

    pause(500);
    gpioTerminate();
    rclcpp::shutdown();
    return 0;
}
