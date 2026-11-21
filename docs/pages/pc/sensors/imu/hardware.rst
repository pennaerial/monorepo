:doc-status: evolving
:status-reason: ICM20948 does not populate the common IMU cache; LSM6DSO supplies samples but full hardware Payload integration and shared wiring remain unfinished.

IMU on Hardware
===============

Here **I2C** is the chip-to-sensor bus: **SDA** carries data and **SCL** supplies
its clock. **GPIO** means a general-purpose chip pin; a power-enable pin switches
on the sensor's supply. **FreeRTOS** schedules the read task on the physical board.
**CMake** selects which driver source is compiled; its cache saves that selection.

The IMU component selects one backend using CMake's ``IMU_MODEL`` cache variable:
``icm20948`` (default) or ``lsm6dso``. This choice is independent of the ESP chip
target. From ``payload_controller/``, build the target once, then choose a backend:

.. code-block:: bash

   make esp32s3
   idf.py -B build/esp32s3 -D SDKCONFIG=sdkconfig.esp32s3 -D IMU_MODEL=lsm6dso reconfigure
   make esp32s3
   idf.py -B build/esp32s3 -D SDKCONFIG=sdkconfig.esp32s3 -p /dev/ttyACM0 flash monitor

Replace the port with your device's port. To switch back, reconfigure with
``-D IMU_MODEL=icm20948`` and rebuild. The choice persists in that build cache;
``make esp32s3`` alone does not reset it. See :doc:`../../build-system`.

Current implementation differences
----------------------------------

**LSM6DSO:** enables the sensor power supply, creates I2C controller 0, checks WHO_AM_I,
configures 104 Hz sensor output (104 measurements per second), and runs a roughly 100 Hz FreeRTOS acquisition
task. Each burst reads gyro and accelerometer axes, decodes signed little-endian
16-bit values, scales them into rad/s and m/s², stamps the reading, and writes the
common IMU cache. The message marks orientation as unavailable; the EKF predicts
its own orientation. Failed transactions are bounded by a timeout.

**ICM20948:** ``start()`` configures the device and ``update()`` polls registers.
The current code stores scaled acceleration, gyro, and temperature in internal
arrays, but does not call ``write_latest``. A successful device identity check
therefore does not mean new common IMU samples are reaching Payload. Unit/frame
conversion and cache publication need completing before using this backend with
the integrated estimator.

Wiring and first observations
-----------------------------

The current IMU sources use SDA GPIO 3, SCL GPIO 4, and power-enable GPIO 7.
LSM6DSO expects address ``0x6B`` and identity ``0x6C``. These are source-code
assumptions to compare with the actual board revision. The motor backend also
assigns GPIO 7 to the right bridge: reconcile that conflict before enabling both.

Start with the IMU-only application. Check its identity log, upright gravity
reading, and the sign of each axis during a simple hand rotation. Confirm units,
board-to-body axis mapping, timestamp progression, and sample freshness before
connecting readings to the EKF. Full hardware Payload startup is currently absent.

Sources: `LSM6DSO <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/lsm6dso.cpp>`_
and `ICM20948 <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/icm20948.cpp>`_.

How the physical read becomes a measurement
-------------------------------------------

I2C is a two-signal bus: the controller addresses a device, writes a register
number, and reads/writes bytes at that location. A register is a numbered device
setting or result. “WHO_AM_I” is a fixed identity register used to check that the
expected chip answered; it does not measure motion.

For LSM6DSO, ``initialize`` adds the device, checks identity, enables coherent
multi-byte reads and address auto-increment, then configures the accelerometer
and gyro. Failed initialization removes the device handle. ``read_sample`` reads
12 bytes starting at register 0x22: six gyro bytes followed by six accelerometer
bytes. Each axis uses two little-endian bytes, meaning the least significant
byte comes first. The code joins them into a signed 16-bit number.

At the configured ranges, each accelerometer count is multiplied by
``0.000061 * 9.80665`` to obtain m/s². Each gyro count is multiplied by
``0.00875 * pi / 180`` to obtain rad/s. Changing the measurement range also requires
changing these scale factors. ``make_imu_message`` adds monotonic time since boot,
uses frame name ``imu_link``, and sets orientation covariance's first element to
-1 to indicate that orientation was not measured.

``read_task`` is a C-compatible entry point into ``read_loop``. That loop reads,
publishes successful samples to the local cache, and uses ``xTaskDelayUntil`` to
schedule against the previous wake time. Transaction time therefore does not
simply add to every 10 ms sampling interval. Failed reads log an error and do not
replace the cache with a fake new reading.

ICM20948 uses register banks: ``switch_bank`` changes which group of registers
subsequent addresses refer to. Startup powers the chip, opens address 0x69 at
100 kHz, resets/wakes it, enables accelerometer/gyro axes, and logs identity.
``get_sensor_data`` reads 14 bytes with high byte first, joins seven signed values,
and scales three acceleration axes, three gyro axes, and temperature.
The divisors 16384 and 131 yield g and degrees/second for the assumed ranges,
not the EKF's m/s² and rad/s. The temperature expression is ``raw/333.87 + 21``.
Those internal arrays are not yet converted into the common cached message.
Unlike the bounded LSM6DSO operations, these ICM I2C helpers use an infinite
transaction timeout and ``ESP_ERROR_CHECK``, which aborts on an error. This is
another difference to account for when completing that backend.
