:doc-status: evolving
:status-reason: Latest-value sampling has no full time synchronization or delayed-state replay, and the camera cache has no timestamp.

EKF Integration and Timing
==========================

Payload collects fresh measurements into ``EKF::Measurements`` and calls
``EKF::update`` from one owning thread. Keep all filter-state reads and writes
on that thread; the singleton does not provide synchronization. Optional fields
allow a pass to proceed without waiting for every sensor.

How fresh samples are assembled
-------------------------------

* An independent IMU cursor supplies timestamped gyro and specific force; finite
  checks reject invalid vectors. The last valid gyro is also retained for wheels.
* Left/right speed cursors update pending wheel values independently. Payload
  waits until both sides and a valid gyro have arrived, then submits one encoder
  sample and clears those pending flags. Arrival in different loop iterations is
  supported, but this is not exact timestamp synchronization or an age bound.
* A camera cursor supplies a new world-position vector independently. Camera
  correction does not wait for a fresh wheel pair or IMU prediction.

``EKF::update`` applies IMU prediction, then encoder correction, then camera
correction for the supplied snapshot. Logging and motor feedback have their own
cursors and do not consume estimator readings. See :doc:`../payload`.

Which clock drives which operation?
-----------------------------------

IMU prediction uses sensor timestamp differences in **nanoseconds**, converted to
seconds. The first sample seeds time without integrating an unknown preceding
interval. A greater timestamp predicts; an equal timestamp does not; a backwards
timestamp reseeds the clock without integrating backwards. Resetting simulation
time does not automatically reset the nominal state or covariance.

Encoder SITL uses simulation-time windows. Motor control uses monotonic wall time
between consumed velocity samples. Camera samples are generated at simulated
10 Hz but arrive in the EKF as a Vector3 without timestamp. The overall application
loop requests a 100 ms delay and retains only latest cached readings, so it can
skip intermediate IMU samples. These are important limits for fast motion or
simulation running faster/slower than real time.

Debug in data-flow order
------------------------

1. Check raw Gazebo topics or hardware acquisition before changing filter equations.
2. Verify timestamps, units, body/world frames, and signs. A gravity/frame mistake
   often looks like rapidly growing position error.
3. Check cache update counters, both encoder pending flags, and EKF log
   ``input[imu,enc,cam]`` to learn which measurements actually reached the filter.
4. Verify initialization matches the simulated pose and physical sensor offsets.
5. Only then evaluate covariance/noise settings described in :doc:`ekf`.

The current API exposes estimated position through ``get_position()``; no
estimator-pose DDS topic or position feedback controller is wired here. Add an
explicit owner-safe output path if another subsystem needs more state.

Source: `Payload estimator integration <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/payload/payload.cpp>`_.
