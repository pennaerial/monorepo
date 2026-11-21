:doc-status: evolving
:status-reason: Camera readings are generated from Gazebo truth plus synthetic noise/bias, rather than a real visual localization pipeline.

Camera Position in SITL
=======================

This backend belongs to the ``linux`` simulation target. **Ground truth** is the
position known by the simulator before sensor noise is added. A **pose** combines
position and orientation; SceneBroadcaster is the Gazebo system that publishes
poses. **10 Hz** means ten samples per simulated second.

``SimCamera`` listens to ``/world/<world>/pose/info`` from SceneBroadcaster,
selects the configured model, and takes at most one position sample per 100 ms
of simulation time. Each axis receives 0.05 m standard-deviation white noise
and a fixed random bias drawn with 0.02 m standard deviation. **Standard deviation**
describes the spread of random errors; **bias** is a persistent offset. A fixed **seed** (42), the starting value for the random generator,
makes the random sequence repeatable for a matching callback sequence.

It writes those synthetic positions into ``Camera::write_latest``. The current
model origin and IMU origin are treated as coincident. This is a position-sensor
stand-in, not evidence that a camera can measure absolute position this accurately.

Run :doc:`../../getting-started`, inspect ``CAMERA_SITL`` logs, and compare the
paired ground-truth and camera XYZ values with the EKF state logs. A persistent
camera bias should not be mistaken for uncorrelated white noise; the current
camera model in the filter has no separate camera-bias state.

If no updates arrive, check the world/model names, message header timestamps,
and SceneBroadcaster system in ``gz-models/server.config``. Then trace
``write_latest -> Payload::update_estimator -> EKF::camera_adjustment``.

Source: `camera_sitl.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/camera/camera_sitl.cpp>`_.
