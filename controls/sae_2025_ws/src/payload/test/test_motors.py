#!/usr/bin/env python3
"""
DRV8835 motor test script (PHASE/ENABLE mode — MODE pin must be wired to 3.3V)

Motor A (left):  APHASE=BCM16, AENABLE=BCM13 (hardware PWM1)
Motor B (right): BPHASE=BCM15, BENABLE=BCM18 (hardware PWM0)

Run:  python3 test_motors.py
"""

import lgpio
import time

# --- Pin assignments (BCM numbering) ---
A_PHASE  = 16   # Motor A direction (APHASE)
A_ENABLE = 13   # Motor A speed    (AENABLE / PWM1)
B_PHASE  = 15   # Motor B direction (BPHASE)
B_ENABLE = 18   # Motor B speed    (BENABLE / PWM0)

PWM_HZ   = 200
TEST_SPEED = 0.5   # 50% duty cycle


def set_motor(h, phase_pin, enable_pin, speed):
    """speed: -1.0 to 1.0  (negative = reverse, 0 = stop)"""
    lgpio.gpio_write(h, phase_pin, 1 if speed < 0 else 0)
    lgpio.tx_pwm(h, enable_pin, PWM_HZ, abs(speed) * 100.0, 0, 0)


def stop_all(h):
    for enable in [A_ENABLE, B_ENABLE]:
        lgpio.tx_pwm(h, enable, PWM_HZ, 0.0, 0, 0)
    for phase in [A_PHASE, B_PHASE]:
        lgpio.gpio_write(h, phase, 0)


# --- Open GPIO chip (try RPi 5 first, fall back to RPi 4) ---
h = None
for chip in [4, 0]:
    try:
        h = lgpio.gpiochip_open(chip)
        print(f"Opened gpiochip{chip} (handle={h})")
        break
    except lgpio.error as e:
        print(f"gpiochip{chip} failed: {e}")

if h is None:
    print("ERROR: Could not open any gpiochip — check /dev/gpiochip* permissions")
    exit(1)

# --- Claim outputs ---
for label, pin in [("A_PHASE", A_PHASE), ("A_ENABLE", A_ENABLE),
                   ("B_PHASE", B_PHASE), ("B_ENABLE", B_ENABLE)]:
    rc = lgpio.gpio_claim_output(h, pin, 0)
    print(f"  ClaimOutput {label} (BCM{pin}): rc={rc}")

# --- Start PWM at 0 duty so they're configured before use ---
lgpio.tx_pwm(h, A_ENABLE, PWM_HZ, 0.0, 0, 0)
lgpio.tx_pwm(h, B_ENABLE, PWM_HZ, 0.0, 0, 0)

print(f"\nPins: A_PHASE=BCM{A_PHASE} A_ENABLE=BCM{A_ENABLE} "
      f"B_PHASE=BCM{B_PHASE} B_ENABLE=BCM{B_ENABLE}")
print(f"PWM: {PWM_HZ} Hz  Speed: {int(TEST_SPEED*100)}%\n")

try:
    print("=== Motor A — FORWARD (2s) ===")
    set_motor(h, A_PHASE, A_ENABLE, TEST_SPEED)
    time.sleep(2)
    stop_all(h)
    time.sleep(0.5)

    print("=== Motor A — REVERSE (2s) ===")
    set_motor(h, A_PHASE, A_ENABLE, -TEST_SPEED)
    time.sleep(2)
    stop_all(h)
    time.sleep(0.5)

    print("=== Motor B — FORWARD (2s) ===")
    set_motor(h, B_PHASE, B_ENABLE, TEST_SPEED)
    time.sleep(2)
    stop_all(h)
    time.sleep(0.5)

    print("=== Motor B — REVERSE (2s) ===")
    set_motor(h, B_PHASE, B_ENABLE, -TEST_SPEED)
    time.sleep(2)
    stop_all(h)
    time.sleep(0.5)

    print("=== Both FORWARD (2s) ===")
    set_motor(h, A_PHASE, A_ENABLE, TEST_SPEED)
    set_motor(h, B_PHASE, B_ENABLE, TEST_SPEED)
    time.sleep(2)
    stop_all(h)

    print("\nAll tests complete.")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    stop_all(h)
    lgpio.gpiochip_close(h)
    print("GPIO closed.")
