:doc-status: evolving
:status-reason: DRV8833 pin direction remains unverified, GPIO 7 overlaps the IMU power enable, and the ESP entry point does not start motors.

Motors on Hardware
==================

``DRV8833`` drives both H-bridges using LEDC PWM. The bridges share a 20 kHz
timer with 10-bit duty resolution.

The source's current bridge mapping is:

.. list-table:: Source assumptions, not a validated wiring guide
   :header-rows: 1

   * - Side
     - IN1 / IN2
     - LEDC channels
   * - Left
     - GPIO 10 / 8
     - 0 / 1
   * - Right
     - GPIO 6 / 7
     - 2 / 3

Both inversion flags are currently false with a TODO to verify wheel direction.
GPIO 7 is also the IMU power-enable pin in the current IMU sources; resolve board
revision/pin allocation before enabling both systems.

From duty to electrical output
------------------------------

``start()`` configures the shared timer once and initializes both bridge inputs
low (coast). ``publish()`` ignores calls before startup, converts non-finite input
to zero, clamps magnitude, and chooses direction. In slow-decay operation, one
input stays high while the other is PWM driven with inverse drive fraction.
A commanded zero results in both inputs high (brake), unlike the initial coast
state. The LEDC full-duty value here is ``1 << 10``.

The ESP application currently starts only the IMU, so flashing the default
application does not exercise motors. A future bench integration needs validated
pin allocation, direction checks at low duty with wheels clear, encoder feedback,
and a stale-feedback policy before enabling the full speed loop. Use
:doc:`../../getting-started` for the supported flash/monitor commands, and
:doc:`../../payload` for the integration point.

Source: `DRV8833.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/DRV8833.cpp>`_.
Further learning: `TI DRV8833 datasheet <https://www.ti.com/lit/ds/symlink/drv8833.pdf>`_.
