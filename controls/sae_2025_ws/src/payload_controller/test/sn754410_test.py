import pigpio  # pyright: ignore[reportMissingImports]
import time

# ----------------------
# Pin Definitions (BCM)
# ----------------------

A_PWM = 18
A_IN1 = 15
A_IN2 = 14

B_PWM = 13
B_IN1 = 16
B_IN2 = 20

PWM_FREQ = 1000  # Hz
PWM_DUTY = 128  # ~50% of default range 0-255

# ----------------------
# Setup
# ----------------------

pi = pigpio.pi()
if not pi.connected:
    print("ERROR: Could not connect to pigpiod — is the daemon running? (sudo pigpiod)")
    exit(1)

# Claim direction pins as outputs
for pin in [A_IN1, A_IN2, B_IN1, B_IN2]:
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)

# Claim PWM pins as outputs
for pin in [A_PWM, B_PWM]:
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)

# ----------------------
# Motor Control Functions
# ----------------------


def motor_forward(pwm, in1, in2):
    pi.write(in1, 1)
    pi.write(in2, 0)
    pi.set_PWM_frequency(pwm, PWM_FREQ)
    pi.set_PWM_dutycycle(pwm, PWM_DUTY)


def motor_reverse(pwm, in1, in2):
    pi.write(in1, 0)
    pi.write(in2, 1)
    pi.set_PWM_frequency(pwm, PWM_FREQ)
    pi.set_PWM_dutycycle(pwm, PWM_DUTY)


def motor_stop(pwm, in1, in2):
    pi.set_PWM_dutycycle(pwm, 0)
    pi.write(in1, 0)
    pi.write(in2, 0)


# ----------------------
# Test Sequence
# ----------------------

try:
    print("Motor A forward")
    motor_forward(A_PWM, A_IN1, A_IN2)
    time.sleep(2)

    print("Motor A reverse")
    motor_reverse(A_PWM, A_IN1, A_IN2)
    time.sleep(2)

    print("Motor A stop")
    motor_stop(A_PWM, A_IN1, A_IN2)
    time.sleep(1)

    print("Motor B forward")
    motor_forward(B_PWM, B_IN1, B_IN2)
    time.sleep(2)

    print("Motor B reverse")
    motor_reverse(B_PWM, B_IN1, B_IN2)
    time.sleep(2)

    print("Motor B stop")
    motor_stop(B_PWM, B_IN1, B_IN2)

finally:
    # Cleanup
    pi.set_PWM_dutycycle(A_PWM, 0)
    pi.set_PWM_dutycycle(B_PWM, 0)
    pi.stop()
    print("Done.")
