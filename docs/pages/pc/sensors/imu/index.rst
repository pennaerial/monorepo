IMU
===

An inertial measurement unit measures angular velocity with a gyroscope and
specific force with an accelerometer. It does not directly measure position.
Integrating those signals predicts motion, but noise and bias accumulate, which
is why the :doc:`../../estimation/index` filter also uses other sensors.

The common ``drivers::IMU`` interface exposes ``start()``, ``update()``, and a
latest-sample cache using ``sensor_msgs_msg_Imu``. ``instance()`` is supplied by
the selected backend. **SITL** is simulation running on your computer. It receives **callbacks**,
functions called when simulator messages arrive. Hardware may **poll** (actively
ask for a reading) or use a scheduled **task** to acquire samples.
A sample's units and timestamp matter more to the estimator than its C++ type.

A stationary, upright accelerometer normally reports about +9.81 m/s² on its up
axis: it senses support force. The EKF rotates this specific force into world
coordinates and adds downward gravity. Passing gravity-removed acceleration into
that same model would subtract gravity twice.

.. toctree::
   :maxdepth: 1

   sitl
   hardware

Next: :doc:`../encoders/index` and :doc:`../../estimation/integration`.
