Feedforward
===========

Feedforward estimates duty from the requested speed before waiting for an error.
It is a model of expected effort, so changing the load or battery can leave an
error for feedback to correct. It is not an estimate of the robot's position.

``FeedForward::update(dt)`` returns zero when ``dt <= 0`` or the setpoint is zero.
Otherwise its current model is:

.. math::

   u_{ff} = k_F\operatorname{sign}(\omega_{set})
          + k_V\frac{\omega_{set}}{\omega_{scale}}

The constant signed term approximates friction; the linear term approximates the
voltage needed for speed. ``dt`` validates the call but does not otherwise enter
this static model. There is no acceleration feedforward term.

Payload currently uses ``k_F=0.05``, ``k_V=0.95``, and
``omega_scale=7.434 rad/s``. At +7.434 rad/s the model predicts duty 1 before
feedback; at -7.434 it predicts -1. A zero scale would make the calculation invalid,
so calibration must provide a nonzero speed. This class does not clamp output;
:doc:`pwmcontroller` combines and clamps the total request.

Calibration and observation
---------------------------

Use :doc:`../getting-started` to run the simulation. Compare requested speed,
encoder speed, and duty. If a steady operating point needs substantial PI effort,
the feedforward estimate may not match the load. Change one parameter at a time
and retain a repeatable test condition.

The full-duty calibration in ``payload.cpp`` does not configure the electrical
constants in :doc:`../sensors/motors/sitl`; those define a separate simulated plant.
Hardware calibration requires completed encoder acquisition and validated motor
wiring first. Keep provisional calibration changes marked in progress.

Source: `feedforward.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/feedforward.cpp>`_.
Next: :doc:`pid`.
