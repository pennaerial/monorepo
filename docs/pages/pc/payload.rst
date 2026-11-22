:doc-status: evolving
:status-reason: Payload currently connects the simulation path; setpoints are fixed and full hardware startup is not enabled.

The Payload Class
=================

``Payload`` connects the drivers to the estimator, controllers, and DDS debug
publishing. It owns left/right motor
controllers and wrappers, reads shared sensor caches, passes measurements to the
EKF, and publishes DDS debug messages. Read :doc:`overview` for the complete loop.

Initialization and ownership
----------------------------

``Payload::instance()`` returns a pointer to one shared static instance.
The constructor pairs each wheel's motor and encoder with ``Side::LEFT`` or
``Side::RIGHT`` and constructs a speed controller for each. ``init(start_position)``
resets the EKF, starts the camera and IMU, then starts both encoders and motors.
The caller supplies the world-frame IMU position; attitude starts as identity.

The current ``app_main()`` calls this only on the ``linux`` simulation target. Do not enable the whole
hardware loop by copying startup blindly: the hardware encoder and camera
producers are incomplete, and wiring must be reconciled across drivers.

One update, in order
--------------------

1. ``update_estimator()`` collects new IMU, encoder, and camera snapshots, then
   calls ``EKF::update()``. Sensors are optional inputs; the estimator can update
   with only the measurements that arrived.
2. ``publish_sensor_debug()`` reads fresh IMU/count data and publishes DDS IMU
   and MotorState messages. Encoder count messages can update when either side
   changes, carrying the last known count for the other side.
3. ``motor_updates()`` reads encoder velocity and computes the next duty for each
   wheel. The current setpoints are constants: left 7, right 3 rad/s.

Why separate sample cursors matter
----------------------------------

A cursor is the update counter last seen by one consumer. The estimator, motor
feedback, and debug publishing each keep their own cursor. Reading a sample for
logging therefore cannot accidentally hide it from control. A cache keeps only
the latest value, not every intermediate sample; this is not a lossless queue.

Encoder count and velocity cursors are also distinct. Counts may arrive before
the velocity averaging window is complete. EKF integration waits for both wheel
velocities and a valid gyro; camera correction remains independent. See
:doc:`estimation/integration` for the exact pairing policy.

Controller timing
-----------------

``update_motor()`` uses monotonic wall-clock microseconds between newly consumed
velocity samples. Its first sample seeds time and resets the integral via a stop
setpoint transition. Subsequent samples invoke ``PWMController::command(dt, speed)``.
Between fresh readings it republishes the previous duty without integrating stale
feedback. There is currently no stale-feedback timeout that automatically clears
that duty; this is an implementation limitation to address before broader hardware use.

The controller's wall time and EKF's sensor time are different clocks. Simulation
pause or slow real-time factor can affect them differently. Keep this distinction
in mind when evaluating :doc:`control/index` and :doc:`estimation/index`.

Lookup: :doc:`reference/runtime`.
Source: `payload.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/payload/payload.cpp>`_.
