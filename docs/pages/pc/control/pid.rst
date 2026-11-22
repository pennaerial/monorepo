PID and PI Feedback
===================

Feedback adjusts effort using measured error ``e = setpoint - reading``.
Proportional control reacts to today's error, integral control accumulates a
persistent error, and derivative control reacts to changing measured speed.

For a positive elapsed time, ``PID::update`` calculates:

.. math::

   P &= k_P e \\
   I_{new} &= \operatorname{clamp}(I_{old}+k_I e\Delta t,-I_{max},I_{max}) \\
   D &= -k_D\frac{reading-reading_{previous}}{\Delta t} \\
   u_{feedback} &= P+I+D

Derivative is taken on the measurement to avoid a direct derivative jump when
the setpoint changes. The first reading seeds history without a derivative.
Non-positive ``dt`` returns zero. The constructor argument order is
``PID(kP, kD, kI, setPoint, integralMax)``; note that D precedes I in this API.

PWMController constructs PID with ``kD=0``, so the active wheel loop is PI.
Payload currently uses ``kP=0.08``, ``kI=0.15``, and maximum integral duty 0.3.
The integral limit is a duty contribution, not seconds or an error magnitude.
``new_set_point`` clears accumulated integral when a moving setpoint transitions
to approximately zero (threshold 0.001); it does not reset all history on every
setpoint change.

Why anti-windup exists
----------------------

An actuator cannot exceed full duty. If integral effort keeps growing while the
output is saturated, it can take a long time to unwind after the error changes.
The ``antiWindup`` argument supports a second calculation that restores the
previous integral without accumulating the same sample twice. PWMController
chooses when to invoke that path, as explained in :doc:`pwmcontroller`.

Learn by observing
------------------

Start in :doc:`../getting-started` simulation. Observe error and output before
changing gains. Higher P reacts more strongly but may oscillate with delay.
I can remove persistent error but also overshoot and saturate. The encoder's
100 ms velocity window and wall-clock control timing are part of the system;
tuning gains cannot remove a unit, sign, or stale-sample bug.

Source: `pid.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/pid.cpp>`_.
Further learning: `Åström and Murray, Feedback Systems, PID chapter
<https://fbswiki.org/wiki/index.php/FBS>`_.
