#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>

#include <chrono>
#include <cstdio>
#include <thread>

#include "payload/motor.hpp"

// ---- Pins from payload_params.yaml ----
static constexpr int APHASE  = 16;
static constexpr int AENABLE = 13;
static constexpr int BPHASE  = 15;
static constexpr int BENABLE = 18;

static void pause(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// ---- Test sequences ----

struct Step {
    const char* label;
    float a_speed;   // Motor A: -1.0 to 1.0
    float b_speed;   // Motor B: -1.0 to 1.0
    int   duration_ms;
};

static const Step STEPS[] = {
    // --- Motor A sweep ---
    {"Motor A  FWD 25%",    0.25f,   0.0f,  2000},
    {"Motor A  FWD 50%",    0.50f,   0.0f,  2000},
    {"Motor A  FWD 75%",    0.75f,   0.0f,  2000},
    {"Motor A  FWD 100%",   1.00f,   0.0f,  2000},
    {"STOP",                0.0f,    0.0f,   500},
    {"Motor A  REV 25%",   -0.25f,   0.0f,  2000},
    {"Motor A  REV 50%",   -0.50f,   0.0f,  2000},
    {"Motor A  REV 100%",  -1.00f,   0.0f,  2000},
    {"STOP",                0.0f,    0.0f,   500},

    // --- Motor B sweep ---
    {"Motor B  FWD 25%",    0.0f,   0.25f,  2000},
    {"Motor B  FWD 50%",    0.0f,   0.50f,  2000},
    {"Motor B  FWD 75%",    0.0f,   0.75f,  2000},
    {"Motor B  FWD 100%",   0.0f,   1.00f,  2000},
    {"STOP",                0.0f,   0.0f,    500},
    {"Motor B  REV 50%",    0.0f,  -0.50f,  2000},
    {"Motor B  REV 100%",   0.0f,  -1.00f,  2000},
    {"STOP",                0.0f,   0.0f,    500},

    // --- Both motors ---
    {"Both  FWD 50%",       0.50f,  0.50f,  2000},
    {"Both  FWD 100%",      1.00f,  1.00f,  2000},
    {"STOP",                0.0f,   0.0f,    500},
    {"Both  REV 50%",      -0.50f, -0.50f,  2000},
    {"STOP",                0.0f,   0.0f,    500},
};

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

    // Claim output pins before handing to Motor
    for (auto [label, pin] : {std::pair{"APHASE",  APHASE},
                               std::pair{"AENABLE", AENABLE},
                               std::pair{"BPHASE",  BPHASE},
                               std::pair{"BENABLE", BENABLE}}) {
        int rc = lgGpioClaimOutput(h, 0, pin, 0);
        printf("  ClaimOutput %-8s (BCM%2d): rc=%d\n", label, pin, rc);
    }

    Motor motor_a(h, APHASE, AENABLE);
    // Motor motor_b(h, BPHASE, BENABLE);

    std::cout << "MOTOR FORWARD 100%" << std::endl;
    motor_a.set_speed(1.0f);
    pause(2000);

    std::cout << "MOTOR FORWARD 20%" << std::endl;
    motor_a.set_speed(0.2f);
    pause(2000);


    std::cout << "MOTOR REVERSE 100%" << std::endl;
    motor_a.set_speed(-1.0f);

    pause(2000);

    std::cout << "MOTOR REVERSE 20%" << std::endl;
    motor_a.set_speed(-0.2f);
    pause(2000);




    motor_a.set_speed(0.0f);
    printf("\nDone.\n");

    lgGpiochipClose(h);
    rclcpp::shutdown();
    return 0;
}
