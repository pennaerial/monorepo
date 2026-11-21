Simulation Runtime and Message Conversion
=========================================

Here **simulation** means running the ``linux`` build on your computer, with
Gazebo replacing the physical body, motors, and sensors. Linux is the operating
system hosting the program; **SITL** means software-in-the-loop, this style of
testing a controller with simulated devices. **Gazebo Transport** is the message
library that connects the running simulator and the driver callbacks.

Use :doc:`getting-started` for launch commands. This page explains the code behind
those commands so you can change a model or debug a missing sample.

What sim.sh starts
------------------

``scripts/sim.sh`` takes a model name and an optional world name. It verifies the
model SDF, world SDF, and Linux executable exist, then exports Gazebo's resource
path and ``gz-models/server.config``. An **SDF** describes bodies, joints, sensors,
and simulation properties; the server config supplies simulation systems.

It checks for ``/world/<world>/create``. If that service already exists, it reuses
the running world. Otherwise it starts ``gz sim -r`` and waits up to 60 seconds
for the service. ``HEADLESS=1`` starts the server without a GUI and requests
software rendering so rendering-dependent sensors can still run.

The script requests a spawn at Z=0.1 m, using the name ``<model>_0``, and starts
``build/linux/payload_controller.elf`` with that name/world in its environment.
A shell trap stops the Gazebo process group it created when the controller exits.
A reused world is not stopped. The script does not implement model removal or
unique names on every rerun, so an old model can conflict with a later spawn.

Why transport initialization comes first
----------------------------------------

**FreeRTOS** is the controller's small operating system/task scheduler. Its
**Linux port** is the version adapted to run on a Linux computer. It uses
**SIGALRM**, an operating-system notification delivered to a process, to schedule tasks. Gazebo creates
native worker threads that can hold internal locks. If its worker receives that
scheduling signal while holding a lock, another task can become stuck waiting.

``sitl::initialize_transport`` blocks SIGALRM in the caller temporarily, creates
the shared Gazebo transport workers (which inherit that mask), then restores the
caller's previous mask. It aborts if mask changes fail. Call it before any driver
constructs a Gazebo node. This is thread setup, not a change to simulated physics.

``get_config`` reads ``GZ_MODEL`` and ``GZ_WORLD`` once, under a mutex. Missing
values produce warnings and default to ``payload_0`` / ``default``. Later changes
to those environment variables do not reconfigure already-created drivers.

Conversion is copying, not filtering
------------------------------------

``convert.hpp`` / ``convert.cpp`` translate Gazebo **protobuf** messages (data structures generated using Protocol Buffers)
into the C message structs generated from **IDL**, the schema language used to
declare this project's DDS message fields:

* Time copies seconds and converts nanoseconds to the destination integer type.
* Vector3 and Quaternion copy their named coordinates/coefficients.
* Header copies time if present and looks for ``frame_id`` in its key/value data.
  The fixed frame-name buffer is initialized to zero, then filled with a bounded copy.
* IMU combines those conversions and copies each covariance array.
* ``gz_float_v_cpy`` requires the source array length to match the destination's
  compile-time length; an assertion checks that assumption in assertion-enabled builds.

``find_gz_header_data`` returns a pointer into the original Gazebo message's
storage, or null if no matching value exists. Use/copy that value while the source
message is alive. Conversion does not rotate frames, remove bias, integrate
acceleration, or send a DDS packet. Those jobs belong to the driver, EKF, or DDS
path described in their own pages.

Next: :doc:`sensors/imu/sitl`, :doc:`dds_client/topics`, and :doc:`development-tools`.
Source: `SITL code <https://github.com/pennaerial/monorepo/tree/main/payload_controller/components/sitl>`_.
