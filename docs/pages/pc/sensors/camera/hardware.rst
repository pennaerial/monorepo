:doc-status: evolving
:status-reason: Hardware Camera::start() is empty; no hardware position measurements are produced.

Camera Position on Hardware
===========================

In the ``esp32s3`` hardware target, ``Camera::start()`` is an empty function. The common **cache** (storage for the latest reading) exists,
but stays empty until a producer calls ``write_latest``. There is no camera
capture, calibration, image processing, or localization transport in this component.

A future producer must define how image observations become a position estimate,
which coordinate frame that estimate uses, and where the camera sits relative
to the IMU. It must transform into the filter's Z-up world frame, convert to
metres, and publish only valid results. ``Vector3`` holds three coordinates. The current cache has no timestamp,
quality, or covariance fields; delayed or low-quality measurements will need a
richer contract before reliable real-world fusion.

Do not connect raw pixel coordinates or a camera-frame translation directly to
this world-position cache. Start with :doc:`../../estimation/integration` and
match the measurement model before implementing the producer. The :doc:`sitl`
backend can test integration, but cannot validate a real vision algorithm.

Source: `camera.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/camera/camera.cpp>`_.
