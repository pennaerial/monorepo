#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <lgpio.h>
#include <rclcpp/rclcpp.hpp>

#include "payload/servo.hpp"

constexpr int SERVO_PIN = 23;
constexpr float PULSE_MIN = 700.0f;
constexpr float PULSE_MAX = 2300.0f;
constexpr int FREQ = 50;

static void pause(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static int g_handle = -1;

static void on_sigint(int)
{
    std::printf("\nCaught SIGINT -- closing servo and cleaning up\n");
    if (g_handle >= 0) {
        lgGpiochipClose(g_handle);
    }
    rclcpp::shutdown();
    std::exit(0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, on_sigint);

    int h = -1;
    for (int chip : {4, 0}) {
        h = lgGpiochipOpen(chip);
        if (h >= 0) {
            std::printf("Opened gpiochip%d (handle=%d)\n", chip, h);
            break;
        }
        std::printf("gpiochip%d failed: %d\n", chip, h);
    }

    if (h < 0) {
        std::printf("ERROR: Could not open any gpiochip\n");
        rclcpp::shutdown();
        return 1;
    }
    g_handle = h;

    Servo servo(h, SERVO_PIN, FREQ, PULSE_MIN, PULSE_MAX);

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
    lgGpiochipClose(h);
    rclcpp::shutdown();
    return 0;
}
