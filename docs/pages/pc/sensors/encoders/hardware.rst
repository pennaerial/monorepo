:doc-status: evolving
:status-reason: Encoder_HW constructor and start() are TODOs; there is no physical count acquisition or velocity producer yet.

Encoders on Hardware
====================

The hardware build selects ``encoder_hw.cpp``. It creates a left/right
``Encoder_HW`` instance, but its constructor and ``start()`` have TODO bodies.
There are no configured input pins, pulse counter, interrupts, or velocity
sampling task. Building it successfully does not provide measured feedback.

What must be implemented
------------------------

1. Confirm the actual encoder wiring, supply/logic levels, shaft location, and
   counts per revolution for the board/motor revision.
2. Decode direction and counts from the physical signals (for quadrature, the
   relative order of A/B transitions identifies direction).
3. Use a monotonic clock and a documented averaging window to compute output-shaft
   rad/s. Handle zero elapsed time, counter wrap, and direction reversal.
4. Call ``write_latest(ticks, angular_velocity_rad_s)`` with an internally
   consistent sample. Keep consumer cursors independent as in :doc:`../index`.
5. Validate a hand-turned revolution and known rotation direction before wiring
   speed feedback into :doc:`../../control/index`.

The SITL implementation in :doc:`sitl` is the current reference for units and
freshness behavior, but it cannot establish hardware pin assignments. There is
no existing hardware encoder command beyond building/flashing the project;
implementation and a bench acquisition test come first.

Source: `encoder_hw.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/encoder_hw.cpp>`_.
