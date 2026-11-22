:doc-priority: key
:key-reason: Read before driver changes to understand units, sample freshness, caches, and backend contracts.

Sensors and Actuators
=====================

Motors live alongside their encoder feedback because the two form one physical
control loop.
Start with the common contract, then choose a device and its SITL or hardware backend.

How drivers connect to the system
---------------------------------

A backend receives a Gazebo message or reads a physical device, converts it into
application units, and writes a thread-protected latest-value cache. Payload
consumes the cache for estimation, control, and debug publishing. Motor drivers
receive duty commands instead of producing sensor samples.

Each sensor consumer holds its own last-seen update counter. ``get_latest_if_updated``
copies data and advances that consumer's cursor only when something changed.
Mutexes protect snapshots; they do not preserve a history of every sample. See
:doc:`../payload` for why independent cursors prevent logging from consuming control data.

The build selects the backend; application code uses common interfaces.
Initialize Gazebo Transport before constructing Linux drivers. The setup blocks
FreeRTOS's SIGALRM scheduling signal in Gazebo workers to avoid locking problems.
See :doc:`../simulation` for the thread setup and why its order matters.

.. list-table:: Device contracts
   :header-rows: 1

   * - Device
     - Data or command
     - Consumers
   * - IMU
     - Timestamp, body gyro rad/s, specific force m/s²
     - EKF prediction and DDS debug
   * - Encoders
     - Output-shaft counts and speed rad/s, per side
     - Speed feedback, EKF wheel correction, DDS counts
   * - Camera position
     - World XYZ metres
     - EKF position correction
   * - Motors
     - Signed normalized duty [-1,1]
     - SITL torque model or hardware H-bridge

Simulation names are part of the contract: the current model has ``base_link``,
``imu_sensor``, ``front_left_wheel_joint``, and ``front_right_wheel_joint``.
Changing the SDF requires matching changes to backend topic/joint names.
The runtime configuration reads ``GZ_MODEL`` and ``GZ_WORLD`` once, defaulting to
``payload_0`` and ``default``.

.. toctree::
   :maxdepth: 2

   imu/index
   motors/index
   encoders/index
   camera/index

Lookup: :doc:`../reference/drivers`.
