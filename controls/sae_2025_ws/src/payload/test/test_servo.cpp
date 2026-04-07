#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <pigpio.h>
#include <rclcpp/rclcpp.hpp>

#include "payload/servo.hpp"

constexpr int SERVO_PIN = 23;
constexpr float PULSE_MIN = 700.0f;
constexpr float PULSE_MAX = 2300.0f;
constexpr int FREQ = 50;

static void pause(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static void on_sigint(int)
{
    std::printf("\nCaught SIGINT -- closing servo and cleaning up\n");
    gpioTerminate();
    rclcpp::shutdown();
    std::exit(0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, on_sigint);

    int rc = gpioInitialise();
    if (rc < 0) {
        std::printf("ERROR: gpioInitialise() failed (err=%d)\n", rc);
        rclcpp::shutdown();
        return 1;
    }
    std::printf("pigpio initialised (version=%d)\n", rc);

    Servo servo(SERVO_PIN, FREQ, PULSE_MIN, PULSE_MAX);

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
    gpioTerminate();
    rclcpp::shutdown();
    return 0;
}
