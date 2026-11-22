:doc-status: evolving
:status-reason: The camera is a provisional world-position source in SITL; physical image processing and hardware production are absent.

Camera Position
===============

The current camera interface supplies world-frame XYZ position in metres.
It does not deliver images or implement feature detection, visual odometry,
or an AprilTag pipeline. This distinction matters when interpreting estimator accuracy.

``Camera`` caches a ``geometry_msgs_msg_Vector3`` with a mutex and update counter.
Payload reads fresh positions and passes them to the EKF, whose camera measurement
model includes a camera-to-IMU lever arm. That offset is currently zero.

.. toctree::
   :maxdepth: 1

   sitl
   hardware

Next: :doc:`../../estimation/ekf` explains how position corrections constrain drift.
