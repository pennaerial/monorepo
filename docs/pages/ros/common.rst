ROS Common: Shared Framework and UAVs
=====================================

``ros_ws/src/common/`` contains reusable packages used across projects. Start
with the mission and vehicle layers, then follow the package responsible for
your feature. The shared mission framework and UAV path receive the most detail
here; vision, planning, and competition world generation are still evolving.
For build, source, launch, and inspection commands, use :doc:`getting-started`.

System interactions
-------------------

.. mermaid::

   flowchart TB
       accTitle: Shared ROS package interactions
       accDescr: The shared mission loader feeds the UAV manager. Behaviors use the vehicle adapter and PX4 while perception is optional.
       cli["pennair_cli · inspect installed resources"] -.-> config["Mission YAML + parameters"]
       config --> shared["vehicle_common · validate and instantiate modes"]
       shared --> manager["uav · UAVModeManager"]
       manager --> modes["UAV / project modes"]
       modes --> adapter["UAV · Multicopter / VTOL adapter"]
       adapter <-->|"ROS messages through Agent"| px4["PX4 autopilot"]
       px4 <--> sim["sim + Gazebo / physical vehicle"]
       camera["Camera source + vision plugins"] -. "optional perception" .-> modes

A mission manager owns the ROS node and timers. Modes use its vehicle object;
the object translates requests into PX4 messages and keeps the latest telemetry.
Gazebo Transport, ROS topics, and the PX4 XRCE connection serve different roles:
a Gazebo topic visible with ``gz topic`` is not automatically a ROS topic.
Bridges expose selected simulator data to ROS.

Package map
-----------

.. list-table:: Paths relative to ``ros_ws/src/common/``
   :header-rows: 1
   :widths: 24 46 30

   * - Package
     - Responsibility
     - Where to begin
   * - ``vehicle_common``
     - Mission schema, mode registry, lifecycle, shared vehicle/launch helpers.
     - ``mission_loader.py``, ``mode_loader.py``, ``mode_manager.py``.
   * - ``uav``
     - UAV manager, vehicle adapters, reusable behaviors, mission and launch resources.
     - ``UAVModeManager.py``, ``vehicles/``, ``launch/``.
   * - ``pennair_cli``
     - Inspect missions, modes, worlds, and airframes.
     - ``pennair_cli/cli.py`` and ``command/``.
   * - ``sim`` / ``sim_interfaces``
     - Gazebo launch, world generation, entities/scoring, and related ROS services.
     - ``launch/sim2.launch.py``, ``simulation_params.py``, ``world_gen/``.
   * - ``px4_msgs``
     - Generated ROS message definitions matching the PX4 integration.
     - Submodule README and ``msg/``; build before dependent packages.
   * - ``pennair_gz`` / ``pennair_gz_msgs``
     - Custom Gazebo systems/bridges and their protobuf message definitions.
     - Plugin README, ``bridge/``, and CMake files.
   * - ``camera_ros``
     - Camera driver submodule.
     - Its README; capture settings depend on the actual camera.
   * - ``pennair_vision``
     - Vision manager, lazy camera publisher, and plugin implementations (WIP).
     - ``config/vision_manager_params.schema.yaml`` and ``src/plugins/``.
   * - ``pennair_planner``
     - Planner service and straight-line/A* plugins (evolving).
     - ``srv/PlanPath.srv``, ``path_planner.hpp``, ``plugins.xml``.
   * - ``tools``
     - Keyboard payload teleop and AprilTag camera debugging.
     - Package README and installed executable list.
   * - ``sae_gz_cmake``
     - Shared CMake support for Gazebo dependencies.
     - ``CMakeLists.txt`` and exported CMake helpers.

"Common" describes reuse, not a guarantee of maturity. Some common packages also
import interfaces under ``src/sae/``. Build by package dependencies rather than
assuming the three source folders are isolated workspaces.

Shared mission loading and execution
------------------------------------

1. ``RuntimeMission.load_from_path`` reads YAML and validates it with Pydantic.
   Each entry under ``modes`` becomes a ``RuntimeMode`` with a registered mode
   ID, parameters, and transitions. Unknown fields are rejected.
2. ``ModeRegistry`` imports mode modules from ``uav.modes``, ``payload.modes``,
   and ``iarc_flight.modes``. Their ``@register_mode`` decorators supply IDs,
   parameter types, compatible vehicle types, and transition labels. An ID such
   as ``uav.VerticalTakeoffMode`` is a registry key, not an arbitrary import path.
3. Each mode's parameters are validated against its own ``ParamsBase`` subclass.
   The mission computes the intersection of compatible vehicle targets. The
   UAV manager requires a mission compatible with ``UAV``.
4. ``setup_modes`` constructs every mode once and calls
   ``initialize(node, vehicle, params)``. It also stores each transition map.
5. Starting the mission creates a nominal 0.1-second update timer. The UAV
   manager selects the local mode named ``start``. Entering a mode calls
   ``on_enter``; leaving it calls ``on_exit``.
6. Each active update calls ``on_update(dt)`` followed by ``check_status``.
   ``continue`` keeps the mode; a custom outcome such as ``complete`` looks up
   the next local name in that mode's transition map. A target named
   ``terminate`` ends the mission through the manager's termination handler.

Keep ``initialize`` suitable for loading an inactive mode; put activation-specific
work in ``on_enter``. Avoid blocking inside updates because callbacks, telemetry,
and other timers need executor time. ``dt`` is elapsed time, not a promise that
every update arrives exactly 0.1 seconds apart.

The loader validates fields, registry IDs, parameters, and target compatibility;
do not assume it proves the transition graph is complete or every target exists.
Check that ``start`` is present and each possible custom outcome has a valid
transition. The UAV manager handles ``error`` by enabling its failsafe path and
handles ``terminate`` by destroying its node. Other vehicle managers can use the
base class's stop behavior, so termination effects are vehicle-specific.

Read :doc:`../concepts/modes` for the lifecycle and
:doc:`../tutorials/create-mission` for authoring. Tests in
``vehicle_common/test/test_mission_loading.py`` and ``test_mode_loader.py``
cover schema/registry behavior without requiring a full flight simulation.

The UAV path in more detail
---------------------------

``uav_mission`` is the installed executable. It initializes rclpy, constructs
``UAVModeManager``, and spins the ROS executor. The generated parameter module
comes from ``uav/config/uav_mission_parameters.yaml``; update the schema and
rebuild when changing parameter definitions.

``UAVModeManager`` loads the mission, chooses ``Multicopter`` or ``VTOL`` from
``vehicle_class``, and binds modes to that vehicle. Its auto-launch gate waits
for vehicle status, attitude/yaw, local and global position, and PX4's flight
check flag. With ``auto_launch`` false, only the explicit start service starts
the mission loop.

During updates, the UAV manager handles failsafe conditions, sets an origin,
requests arming, waits for needed positions, publishes offboard heartbeats,
and runs the active mode. PX4 still performs the flight-control work. The
manager's failsafe path requests hover/loiter and then landing; read its current
conditions before changing emergency behavior.

The vehicle adapter's responsibilities are deliberately lower-level:

* **Receive telemetry:** callbacks save status, attitude, GPS, and local/global
  position. ``vehicles/UAV.py`` defines the shared state and topic wiring.
* **Send commands:** methods such as ``arm``, ``land``, and
  ``publish_trajectory_setpoint`` construct PX4 messages. A publication is a
  request, not proof the autopilot accepted it; observe status feedback.
* **Convert coordinates:** helpers translate GPS/local goals and manage the
  starting origin. ``NavGPSMode``'s ``LOCAL`` waypoints use local NED relative
  to the starting position. Negative Z is above that origin. Each simulated UAV
  gets its own local origin; equal local coordinates do not imply equal world positions.
* **Specialize by airframe:** ``Multicopter`` and ``VTOL`` adapt behavior to their
  vehicle classes. The airframe registry is in ``vehicles/AirframeClass.py``;
  ``quadcopter`` maps to PX4 autostart 4001 and the ``x500`` model.

A useful first source trace is the installed ``basic.yaml``:
``start`` uses ``VerticalTakeoffMode``; outcome ``complete`` selects ``GPS``;
``NavGPSMode`` reaches its waypoint and selects ``land``; ``LandingMode``
finishes the sequence. Compare the YAML parameters with each mode's schema,
then follow its vehicle method calls rather than starting with all of ``UAV.py``.

Launch files and namespaces
---------------------------

``uav_sitl.launch.py`` resolves a mission name from a package's installed
``missions/`` directory, or falls back to a supplied ``.yaml`` path. It validates
the mission, resolves the airframe, and creates the manager, PX4 process, and
optional simulator/Agent. ``ns_id:=0`` sets both the PX4 instance and ROS namespace
``uav_0``. PX4 is given ``PX4_UXRCE_DDS_NS`` so its messages align with the node.

``standalone:=true`` starts the shared middleware/simulator regardless of the
individual ``run_mw`` and ``launch_sim`` flags. To reuse an existing environment,
set ``standalone:=false`` and then choose those flags deliberately. Avoid duplicate
Agents bound to port 8888 or duplicate simulator instances.

``uavs_sitl.launch.py`` starts one Gazebo world and one Agent, then includes the
single-UAV launch for each vehicle with standalone mode disabled. It defaults
to four vehicles. From a prepared terminal:

.. code-block:: bash

   ros2 launch uav uavs_sitl.launch.py --show-args
   ros2 launch uav uavs_sitl.launch.py num_uavs:=2 \
     mission_package:=uav mission:=basic world:=default

This multi-UAV command starts the default auto-launch behavior once conditions
are satisfied. A comma-separated mission list can assign one mission per UAV;
its length must match ``num_uavs``. Each vehicle needs a distinct namespace and
PX4 instance; the wrapper supplies these.

For one UAV, the node resolves relative ``fmu/...`` names under its namespace.
Current telemetry names include ``fmu/out/vehicle_status_v1`` and
``fmu/out/vehicle_local_position_v1``. Do not silently remove ``_v1`` when copying
commands: the PX4 message version and its exported topic names must match.
See :doc:`getting-started` for inspection commands and QoS troubleshooting.

Simulation, bridges, and resources
----------------------------------

``sim2.launch.py`` loads world files from ``gz-models/worlds`` and model resources
from ``gz-models/models`` using repository environment paths. It launches
``gz sim --render-engine ogre -r``; headless mode adds server/rendering flags and
software-rendering environment settings. It also bridges simulation clock to ROS.
A clock topic does not automatically make every node use simulation time;
inspect the node's ``use_sim_time`` parameter and the code's time source.

``sim/simulation_params.py`` and ``simulations/<world>/<stage>.yaml`` describe
world/stage settings used by world-generation and scoring nodes. These files
are more likely to change with competition needs than the core mode lifecycle.
Use ``ros2 launch sim sim2.launch.py --show-args`` before selecting a stage.

Custom Gazebo systems in ``pennair_gz`` include runtime attachment/detachment.
Its Gazebo requests use custom protobuf types from ``pennair_gz_msgs``;
``sim_interfaces`` contains ROS service types. They are different transports
and schemas. Follow the package's
`plugin README <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_gz/README.md>`_
when changing a bridge or attachment service.

Vision and planning boundaries
------------------------------

.. warning::

   **WIP:** ``pennair_vision``, camera-to-behavior integration, and planner
   integrations are evolving. These packages exist and contain implementations;
   their presence does not mean every mission uses them or their interfaces are final.

``pennair_vision`` owns a manager, lazy camera publisher, and plugins including
basic vision, republishing, and AprilTag processing. Begin with its parameter
schema and plugin registration, then trace the input image and CameraInfo
subscriptions to the output needed by a behavior. ``camera_ros`` supplies a
hardware camera source; the UAV simulation launch can bridge images and camera
info for models marked as camera-equipped. Calibration must match the actual
stream and camera, not just an example file's dimensions.

``pennair_planner`` separates the path-planning service from plugins through
``path_planner.hpp`` and ``plugins.xml``. Straight-line and A* implementations
live in ``src/planners/``. Read ``srv/PlanPath.srv`` first to see the current
request/result contract; trace a mission's actual client before assuming a
planner is part of its control loop. Experimental CV utilities also remain in
``vehicle_common/cv/``; their directory alone does not make them part of the core
mission runtime.

Useful checks after a change
----------------------------

From ``ros_ws/`` with the environment sourced:

.. code-block:: bash

   colcon build --packages-up-to uav
   source install/setup.bash
   colcon test --packages-select vehicle_common uav --return-code-on-test-failure
   colcon test-result --verbose

Normal tests do not replace the live UAV simulation test. Follow :doc:`../ci/checks`
for the separate simulation collection and its prerequisites. For new interfaces,
build both the interface package and consumers; for launch/mission resource edits,
confirm the installed file is updated with ``ros2 pkg prefix --share <package>``.

Sources: `vehicle_common <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/common/vehicle_common>`_,
`UAV package <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/common/uav>`_,
`common packages <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/common>`_.
