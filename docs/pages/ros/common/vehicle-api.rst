:doc-priority: key
:key-reason: Read before changing PX4 requests or telemetry: endpoints, feedback, frames, and units live here.

Vehicle Adapters, PX4 Messages, and Coordinates
===============================================

Modes call a vehicle object instead of constructing every PX4 message themselves.
``vehicle_common.Vehicle`` defines the shared identity/camera metadata and stop
contract; ``uav.vehicles.UAV`` implements the PX4-facing operations. Concrete
``Multicopter`` and ``VTOL`` classes provide specialization.

The command and feedback loop
-----------------------------

.. mermaid::

   flowchart LR
       accTitle: Vehicle command and feedback loop
       accDescr: A mode calls an adapter method, which publishes a typed message through the Agent to PX4. Telemetry callbacks update state used on later mode ticks.
       mode["Mode update"] --> api_call["Vehicle method"]
       api_call --> ros["fmu/in · ROS message"]
       ros --> agent["XRCE Agent"] --> px4["PX4"]
       px4 --> telemetry["fmu/out · telemetry"]
       telemetry --> callback["Adapter callback → latest state"]
       callback --> mode

Publishing a command does not wait for acceptance. The current adapter does not
subscribe to VehicleCommandAck; behavior generally advances by observing status
and position. A log saying "command sent" identifies a publication, not an
acknowledged state change.

Message endpoints
-----------------

All paths below are relative to the node namespace, for example
``/uav_0/fmu/in/vehicle_command``. ``Vehicle.px4_transport_path`` adds ``fmu/``;
it does not use the ``vehicle_name`` string to override ROS name resolution.

.. list-table:: Core endpoints in ``UAV.py``
   :header-rows: 1
   :widths: 38 27 35

   * - Relative topic
     - Message
     - Used for
   * - ``fmu/in/vehicle_command``
     - ``VehicleCommand``
     - Arming, mode selection, takeoff, landing, actuator requests.
   * - ``fmu/in/offboard_control_mode``
     - ``OffboardControlMode``
     - Offboard heartbeat and requested control level.
   * - ``fmu/in/goto_setpoint``
     - ``GotoSetpoint``
     - Position goal with optional speed/heading-rate limits.
   * - ``fmu/in/trajectory_setpoint``
     - ``TrajectorySetpoint``
     - Position or full position/velocity/acceleration reference.
   * - ``fmu/out/vehicle_status_v1``
     - ``VehicleStatus``
     - Arming, navigation state, preflight checks, failsafe flags, IDs.
   * - ``fmu/out/vehicle_attitude``
     - ``VehicleAttitude``
     - Quaternion converted to roll, pitch, yaw.
   * - ``fmu/out/vehicle_local_position_v1``
     - ``VehicleLocalPosition``
     - Latest local position and initial local/GPS reference.
   * - ``fmu/out/vehicle_global_position``
     - ``VehicleGlobalPosition``
     - Global position used by navigation and takeoff.
   * - ``fmu/out/vehicle_gps_position``
     - ``SensorGps``
     - Early coarse global-position fallback.

The adapter also creates ``fmu/in/target_position`` with VehicleLocalPosition;
creating that publisher does not prove PX4 exports a corresponding consumer.
Check PX4's configured DDS topic list when adding or diagnosing an endpoint.
VTOL adds ``fmu/out/vtol_vehicle_status``.

Telemetry callbacks store the most recent messages, not a time-aligned sensor
history. The first local-position callback saves ``local_origin`` and
``gps_origin``. Raw GPS temporarily populates a VehicleGlobalPosition structure;
the global-position callback replaces it and destroys the raw-GPS subscription.
Status callbacks update IDs, navigation/arming state, and combined failsafe state.

Selecting a command API
-----------------------

* ``arm``, ``disarm``, ``hover``, ``engage_offboard_mode``, and ``land`` construct
  VehicleCommand messages. ``stop`` calls ``hover`` for a UAV.
* ``takeoff(height)`` reads current global position and sends a NAV_TAKEOFF goal
  at current altitude plus height. ``attempted_takeoff`` prevents resending it
  through that helper. It is not reset automatically for a second takeoff.
* ``publish_goto_setpoint`` sends a local NED position. Optional limits have
  matching enable flags; passing None leaves those limits unset. PX4 handles
  the resulting motion/trajectory smoothing.
* ``publish_position_setpoint`` sends position in TrajectorySetpoint. With
  ``relative=True`` it adds the current local position each call. Repeating a
  relative request therefore moves the target with the vehicle; calculate a
  fixed destination once if that is the intended behavior.
* ``publish_trajectory_setpoint`` explicitly supplies position, velocity,
  acceleration, and yaw. IARC tracking uses this richer kind of reference.

The heartbeat requests position-level offboard control. It is distinct from the
setpoint message and does not itself select OFFBOARD. Modes must supply suitable
references and request the appropriate PX4 state. See :doc:`flight-modes` for
actual call sequences and the `PX4 offboard guide <https://docs.px4.io/main/en/flight_modes/offboard>`_
for autopilot requirements.

Position-only publishing calculates yaw toward the goal, retaining current yaw
within 3 m to avoid heading changes from small position errors. ``lock_yaw``
explicitly retains a known yaw. It does not generate a complete obstacle-aware
trajectory.

Coordinates with concrete examples
----------------------------------

PX4 local coordinates use **NED**: X north, Y east, Z down. Position is metres,
velocity m/s, and trajectory yaw radians. A goal ``[5,0,-3]`` is 5 m north and
3 m above the local reference. It is not necessarily 5 m ahead of the vehicle's
nose. Gazebo world axes and camera axes must be converted deliberately.

``NavGPSMode`` passes ``LOCAL`` values directly to the adapter; it does not add
``local_origin`` to each waypoint. PX4 defines that reference. The launch/spawn
setup gives each simulated UAV its own reference, so check the reported local
position before comparing goals across vehicles.

``gps_to_local`` uses the stored GPS reference and a local flat-earth
approximation:

.. code-block:: text

   north = Earth radius × latitude difference in radians
   east  = Earth radius × longitude difference in radians × cos(reference latitude)
   down  = -(target altitude - reference altitude)

The actual function accepts ``(latitude, longitude, altitude)`` even though some
older comments describe the order differently. ``local_to_gps`` reverses that
approximation. It is intended for nearby navigation, not global route planning.

``uav_to_local`` rotates XY by current yaw. With ``relative=True`` it then adds
current position; without it, it only rotates the vector. It does not apply a
full roll/pitch rotation. Camera offsets used through this helper are not a
complete camera-to-world calibration pipeline.

``set_origin`` sends a PX4 origin command and rotates the camera offset once.
Its current guard tests ``self.yaw`` for truthiness, so a valid exact zero yaw
can leave ``origin_set`` false. Read that branch if origin setup repeats despite
otherwise valid telemetry; do not assume the docs imply a completed origin reset.

QoS and timestamps
------------------

The main telemetry subscriptions request best effort, transient-local durability,
keep-last depth 10; VTOL status uses depth 1. Match actual publisher settings
when diagnosing missing data. The simple command publishers use a depth-10 ROS
QoS argument. Both sides must also use compatible px4_msgs definitions.

Setpoint/heartbeat timestamps use the node's clock in microseconds. VehicleCommand
uses a new ``Clock()``; mode ``dt`` uses wall time. The system therefore does not
have one uniform time source. A bridged ``/clock`` alone does not make all these
paths pause together; see :doc:`simulation`.

Inspect the live connection
---------------------------

With the single-UAV simulation running:

.. code-block:: bash

   ros2 topic info /uav_0/fmu/out/vehicle_status_v1 --verbose
   ros2 topic echo /uav_0/fmu/out/vehicle_status_v1 \
     --qos-reliability best_effort --qos-durability volatile --once
   ros2 interface show px4_msgs/msg/GotoSetpoint
   ros2 interface show px4_msgs/msg/TrajectorySetpoint

Trace one mode's method call to its message constructor, then the endpoint,
then the telemetry field it uses to confirm progress. That same trace is useful
when implementing a new operation.

Sources: `Vehicle base <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/vehicle_common/vehicle_common/vehicle.py>`_,
`UAV.py <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/uav/uav/vehicles/UAV.py>`_,
`VTOL.py <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/uav/uav/vehicles/VTOL.py>`_.
Further learning: `PX4 ROS 2 coordinate and message conventions <https://docs.px4.io/main/en/ros2/user_guide>`_.
