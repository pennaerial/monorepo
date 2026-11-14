Payload Controller C++ API Reference
======================================

.. toctree::
   :maxdepth: 6

   drivers/index
   dds_client/index

FreeRTOS tasks
--------------

ICM20948 hardware and Linux SITL use the same task table in
``payload_controller/main/payload_controller.cpp``. Higher numbers mean higher
FreeRTOS priority; left and right devices have separate tasks at equal priority.

========================= ======== =================
Task                      Priority Requested period
========================= ======== =================
DDS client                7        10 ms
IMU                       6        10 ms
Left / right encoder      6        10 ms each
EKF                       5        10 ms
PWM controllers           4        10 ms
Left / right motor output 3        10 ms each
Payload telemetry         2        100 ms
========================= ======== =================

Periods are rounded to scheduler ticks, with a minimum of one tick. Each task
blocks between updates, including after an overrun. Sensor acquisition precedes
the EKF, which precedes control; priority determines which ready task runs and
does not impose a strict execution sequence.

The PWM task computes both motor duties from new encoder velocity samples.
Each motor task applies its last duty independently. DDS publications use a
single latest-value mailbox per topic, so network delivery cannot block the
sensor, estimator, or control tasks. A newer unsent publication replaces the
previous one. Gazebo callbacks similarly stage the latest sample for each sensor
task; these caches do not retain every sample at rates above the polling rate.

EKF prediction runs only when a fresh IMU sample is available. Encoder and camera
corrections remain disabled until their measurement placeholders and parameters
are implemented. Hardware encoder acquisition and DDS UART transport also remain
unfinished in their existing drivers.
