UAV Startup, Readiness, and Failsafes
=====================================

This page follows ``uav_mission`` and ``UAVModeManager``. The shared lifecycle is
in :doc:`mode-runtime`; commands and telemetry are in :doc:`vehicle-api`.

From executable to running node
-------------------------------

``uav/setup.py`` installs ``uav_mission = uav.uav_mission:main``. That entry point
initializes rclpy, constructs the manager, and spins its executor so callbacks
and timers can run. Its finally block destroys the node and shuts down rclpy
when appropriate. The simulation launch renames/namespaces it to
``/uav_0/mode_manager``.

Manager construction:

1. Run the base manager constructor, loading generated ROS parameters and
   creating the start service/readiness polling timer.
2. Load ``mode_map`` or the installed ``uav/basic`` mission and require ``UAV``
   in its compatible-target intersection.
3. Read camera offsets, vehicle name, class, and ``servo_only``.
4. Create the relative ``mode_manager/failsafe`` Trigger service.
5. Construct ``VTOL`` when the class is VTOL; otherwise construct ``Multicopter``.
   The current branch also uses that fallback for other accepted class strings;
   an airframe registry entry does not guarantee dedicated support for its class.
6. Initialize all mission modes against that vehicle.

Construction creates publishers/subscribers before telemetry has arrived.
A non-null vehicle object does not mean it is ready to fly.

Parameters and their source
---------------------------

``uav/config/uav_mission_parameters.yaml`` is compiled into a Python parameter
module during the package build. ``load_params`` creates its ParamListener and
reads a parameter snapshot. All of these schema entries are currently read-only:

.. list-table:: Current UAV mission parameters
   :header-rows: 1

   * - Name
     - Default
     - Purpose
   * - ``mode_map``
     - Empty string
     - YAML path; empty selects the bundled basic mission.
   * - ``auto_launch``
     - true
     - Automatically start after the readiness gate.
   * - ``servo_only``
     - false
     - Run modes without the normal arming/position/offboard update branches.
   * - ``vehicle_name``
     - ``uav_0``
     - Vehicle identity used by runtime state and camera metadata.
   * - ``vehicle_class``
     - ``MULTICOPTER``
     - Select the vehicle implementation.
   * - ``camera_mount_offsets``
     - ``[0,0,0]``
     - Three body-relative offset values in metres.

Launch can override defaults when creating the node. Changing YAML after the
node starts does not update its parameter snapshot. Rebuild generated schemas
and restart for a definition change. ``vehicle_name`` does not independently
move a node into a ROS namespace; launch must align both.

What auto-launch waits for
--------------------------

The readiness poll requires vehicle status, attitude, yaw (not None), local
position, global position, and a true preflight-check flag. It then starts the
mission timer. These checks confirm fields are populated; there is no age-based
stale-telemetry check in this gate. Inspect live endpoints when debugging an
apparently ready but disconnected system.

With ``auto_launch:=false``, launch still starts the processes and subscribers.
The manual start service starts the timer without calling that readiness gate:

.. code-block:: bash

   ros2 node info /uav_0/mode_manager
   ros2 param get /uav_0/mode_manager auto_launch
   ros2 service call /uav_0/mode_manager/start_mission std_srvs/srv/Trigger '{}'

Use these with the simulation in :doc:`../getting-started`. A successful
service response means the loop started, not that PX4 armed or accepted a goal.
``servo_only`` does not bypass the separate auto-launch readiness method, so do
not assume setting it alone permits telemetry-free automatic startup.

One execution tick, in source order
-----------------------------------

.. mermaid::

   flowchart TB
       accTitle: UAV manager update order
       accDescr: The UAV manager chooses the start mode, prioritizes failsafes, optionally runs servo-only behavior, handles origin and arming, waits for positions, then publishes heartbeat and updates the active mode.
       tick["spin_once"] --> start["Select start if no active mode"]
       start --> fail{"Failsafe?"}
       fail -->|yes| recovery["Request loiter, then landing; return"]
       fail -->|no| servo{"Servo-only?"}
       servo -->|yes| behavior["Run active mode"]
       servo -->|no| origin["Set origin if needed"]
       origin --> armed{"Armed?"}
       armed -->|no| arm["Handle landing/disarm exit or request arm; return"]
       armed -->|yes| pos{"Positions available?"}
       pos -->|no| wait["Return; await telemetry"]
       pos -->|yes| heartbeat["Publish offboard heartbeat"]
       heartbeat --> behavior

When disarmed, the manager requests arming and returns before normal mode
updates. Two exceptions end the node instead: the active mode is LandingMode
and PX4 has exited AUTO_LAND, or takeoff was already attempted and disarming was
unexpected. This prevents repeatedly rearming after an interrupted takeoff.

Once armed and with positions available, the manager publishes an offboard
heartbeat, runs the active mode, and logs landing state as applicable. PX4
continues to own low-level flight control. Since early-return ticks do not call
``run_active_mode``, the next mode ``dt`` can include time spent waiting.

Failsafe behavior
-----------------

The adapter combines status-reported ``failsafe_px4`` with an explicit
``failsafe_trigger``. The Trigger service sets the latter; the next tick enters
the failsafe branch. A mode returning ``error`` directly sets the combined
``failsafe`` flag. A later vehicle-status callback recomputes that combined flag
from the two source flags, so these two error paths are not identical persistent
latches.

On the failsafe branch, the manager sends a hover/AUTO_LOITER request once and
marks ``emergency_landing``. When PX4 reports AUTO_LOITER or is no longer armed,
it sends a land request, then returns without executing the mode. This is an
implemented recovery sequence, not proof that every failure can safely land.
Observe actual PX4 state rather than treating a logged request as acceptance.

``terminate`` destroys the UAV node directly; it does not call the shared base
handler's stop/deactivate/marker cleanup. Normal landing is expected to be
completed by the behavior before termination. An arbitrary new mode returning
``terminate`` while airborne does not automatically request landing.

Where to investigate a stuck start
----------------------------------

First check launch/environment errors, then Agent/PX4 telemetry, then readiness
fields, then arming status, then mode updates. Logs saying "Mode registered"
only show initialization. "Starting Mission" shows timer start. Actual mode
activation and changing PX4 states are later steps.

Sources: `entry point <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/uav/uav/uav_mission.py>`_,
`UAVModeManager.py <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/uav/uav/UAVModeManager.py>`_,
`parameter schema <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/uav/config/uav_mission_parameters.yaml>`_.
