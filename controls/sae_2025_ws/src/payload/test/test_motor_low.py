#!/usr/bin/env python3
"""
Legacy low-level motor test script.

This script drives two pins per motor and was used for previous wiring.
For SN754410 (PWM + IN1 + IN2), prefer `sn754410_test.py`.

Requires the pigpiod daemon to be running (`sudo pigpiod`).
"""

import pigpio
import time

# --- Pin assignments (BCM numbering) ---
A_IN1 = 16  # Motor A IN1 (PWM1)
A_IN2 = 13  # Motor A IN2

B_IN1 = 15  # Motor B IN1 (PWM0)
B_IN2 = 18  # Motor B IN2

PWM_HZ = 500
TEST_DUTY = 128  # ~50% of default range 0-255
RUN_SECS = 2.0


def coast(pi, in1, in2):
    """Coast: both inputs LOW, outputs float."""
    pi.set_PWM_dutycycle(in1, 0)
    pi.set_PWM_dutycycle(in2, 0)


def forward(pi, in1, in2, duty=TEST_DUTY):
    """Forward/coast: IN1=PWM, IN2=0."""
    pi.write(in2, 0)
    pi.set_PWM_frequency(in1, PWM_HZ)
    pi.set_PWM_dutycycle(in1, duty)


def reverse(pi, in1, in2, duty=TEST_DUTY):
    """Reverse/coast: IN1=0, IN2=PWM."""
    pi.write(in1, 0)
    pi.set_PWM_frequency(in2, PWM_HZ)
    pi.set_PWM_dutycycle(in2, duty)


def coast_all(pi):
    for pin in [A_IN1, A_IN2, B_IN1, B_IN2]:
        pi.set_PWM_dutycycle(pin, 0)


# --- Connect to pigpiod ---
pi = pigpio.pi()
if not pi.connected:
    print("ERROR: Could not connect to pigpiod — is the daemon running? (sudo pigpiod)")
    exit(1)
print(f"Connected to pigpiod")

# --- Claim all four pins as outputs, initially LOW ---
for label, pin in [
    ("A_IN1", A_IN1),
    ("A_IN2", A_IN2),
    ("B_IN1", B_IN1),
    ("B_IN2", B_IN2),
]:
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)
    print(f"  SetMode OUTPUT {label} (BCM{pin})")

print(f"\nPWM: {PWM_HZ} Hz  Duty: {TEST_DUTY}/255  Duration: {RUN_SECS}s\n")

try:
    print("=== Motor A — FORWARD/coast (IN1=PWM, IN2=0) ===")
    forward(pi, A_IN1, A_IN2)
    time.sleep(RUN_SECS)
    coast_all(pi)
    time.sleep(0.5)

    print("=== Motor A — REVERSE/coast (IN1=0, IN2=PWM) ===")
    reverse(pi, A_IN1, A_IN2)
    time.sleep(RUN_SECS)
    coast_all(pi)
    time.sleep(0.5)

    print("=== Motor B — FORWARD/coast (IN1=PWM, IN2=0) ===")
    forward(pi, B_IN1, B_IN2)
    time.sleep(RUN_SECS)
    coast_all(pi)
    time.sleep(0.5)

    print("=== Motor B — REVERSE/coast (IN1=0, IN2=PWM) ===")
    reverse(pi, B_IN1, B_IN2)
    time.sleep(RUN_SECS)
    coast_all(pi)
    time.sleep(0.5)

    print("\nAll tests complete.")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    coast_all(pi)
    pi.stop()
    print("GPIO closed.")
