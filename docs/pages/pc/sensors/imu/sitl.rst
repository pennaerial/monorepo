IMU in SITL
===========

``imu_sitl.cpp`` subscribes to:

.. code-block:: text

   /world/<world>/model/<model>/link/base_link/sensor/imu_sensor/imu

``IMU_SITL::on_imu_msg`` converts ``gz::msgs::IMU`` using ``gz_to_dds`` in
``components/sitl/convert.cpp``, then calls ``write_latest``. Conversion copies
timestamps, vectors, quaternion, and covariance fields; it is not a fusion algorithm.
The resulting C struct remains a local cached value until Payload optionally
publishes it through DDS.

Run and observe
---------------

Follow :doc:`../../getting-started` to launch the Agent and controller manually.
Then, in another terminal:

.. code-block:: bash

   gz topic -l
   gz topic -e -t /world/sae/model/payload_2/link/base_link/sensor/imu_sensor/imu

Check that simulation is running, timestamps advance, and values change as the
model moves. Compare the backend's startup topic log with the actual topic list.
If Gazebo publishes but Payload logs do not change, inspect the conversion and
cache; if neither changes, inspect model/sensor configuration first.

Follow one reading through ``on_imu_msg -> write_latest -> Payload::update_estimator
-> EKF::update``. That trace connects :doc:`../index` to
:doc:`../../estimation/ekf`. The EKF uses gyro and specific force, not the incoming
orientation quaternion, for its prediction.

Source: `IMU SITL backend <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/imu_sitl.cpp>`_.
Further learning: `Gazebo sensors tutorial <https://gazebosim.org/docs/harmonic/sensors/>`_.
