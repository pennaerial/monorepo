Launch Architecture, Namespaces, and Hardware
=============================================

Launch files assemble processes and pass configuration; the mode manager then
runs the mission. This page explains that boundary. For a first single-UAV run,
use :doc:`../getting-started`.

Single-UAV simulation: what starts
----------------------------------

``uav_sitl.launch.py`` reads declared launch strings in an OpaqueFunction,
resolves/validates the mission, looks up the airframe, and constructs actions:

* A ``uav_mission`` node renamed ``mode_manager`` under ``uav_<ns_id>``. It receives
  the resolved YAML path, matching vehicle name, airframe class, and auto-launch flag.
* A PX4 process running ``build/px4_sitl_default/bin/px4 -i <ns_id>`` in the PX4
  checkout. Environment selects its model, world, pose, autostart ID, and DDS namespace.
* Optionally one ``MicroXRCEAgent udp4 -p 8888`` and the shared
  ``sim/sim2.launch.py``.
* For models marked camera-equipped in ``uav/utils.py``, a camera image/info
  bridge remapped under the vehicle namespace.

Launch starts these actions; their appearance in a list does not establish
application readiness. PX4 startup, ROS discovery, and the manager's readiness
gate happen afterward.

The most useful single-UAV arguments
------------------------------------

.. list-table:: Current defaults and effects
   :header-rows: 1

   * - Argument
     - Default
     - Effect
   * - ``mission_package`` / ``mission``
     - ``uav`` / ``basic``
     - Installed mission selection; a valid YAML path is the fallback.
   * - ``ns_id``
     - ``0``
     - ROS namespace and PX4 instance number.
   * - ``airframe``
     - ``quadcopter``
     - Registry lookup by alias, numeric ID, or model name.
   * - ``world`` / ``model_pose``
     - ``default`` / empty
     - Gazebo world and optional spawn pose.
   * - ``auto_launch``
     - true
     - Pass the automatic-start flag to the manager.
   * - ``standalone``
     - true
     - Force simulator and middleware startup.
   * - ``run_mw`` / ``launch_sim``
     - false / true
     - Individual startup choices when standalone is false.
   * - ``headless`` / ``debug``
     - false / false
     - GUI/rendering choice and selected log verbosity.

``standalone`` is OR-ed with each individual startup flag. Setting
``run_mw:=false`` alone does not disable the Agent while standalone remains true.
Inspect options without starting the stack:

.. code-block:: bash

   ros2 launch uav uav_sitl.launch.py --show-args
   ros2 run pennair_cli pennair airframe ls

The airframe registry maps ``quadcopter`` to autostart 4001/model ``x500``.
Adding a registry entry does not create a PX4 airframe script or Gazebo model;
all three must agree. Some enumerated classes currently fall back to the
Multicopter adapter; see :doc:`uav-runtime`.

Names must line up across layers
--------------------------------

``ns_id:=2`` gives ROS namespace ``uav_2`` and PX4 instance ``-i 2``. Launch also
sets ``PX4_UXRCE_DDS_NS=uav_2``. The manager's relative ``fmu/...`` topics then
resolve to the same ROS namespace as PX4's messages.

The Gazebo model name is separate: PX4 commonly spawns ``<model>_<instance>``
such as ``x500_2``. Camera bridging constructs Gazebo paths with that model name,
then remaps their ROS outputs to ``/uav_2/camera`` and ``/uav_2/camera_info``.
An image under an x500 model name is not expected to have the same raw topic
path as the ROS node namespace.

``vehicle_name`` is a node parameter; ``namespace`` is a launch property. Setting
only one can break resource identity or message resolution. Use
``ros2 node info /uav_2/mode_manager`` and topic endpoint inspection to check the
resolved names instead of guessing from configuration strings.

Shared environment and multiple UAVs
------------------------------------

``uavs_sitl.launch.py`` launches one world and Agent, then includes the single
launch repeatedly with standalone, middleware, and simulator startup disabled.
It defaults to four vehicles and 5 m spacing along world X. A single mission name
is duplicated; a comma-separated list must have one entry per vehicle.

.. code-block:: bash

   ros2 launch uav uavs_sitl.launch.py num_uavs:=2 \
     mission_package:=uav mission:=basic world:=default

That command uses automatic mission startup. The current multi-UAV wrapper does
not expose the single launch's ``auto_launch`` argument. Supplying an undeclared
argument to the wrapper is not a reliable way to pass it through.

To add one UAV to an already running matching Gazebo/Agent environment:

.. code-block:: bash

   ros2 launch uav uav_sitl.launch.py ns_id:=1 world:=default \
     standalone:=false run_mw:=false launch_sim:=false auto_launch:=false

Choose an unused instance ID. Stop old instances before reusing their ports or
model names. Each PX4 local reference belongs to that vehicle, not the shared
world frame; see :doc:`vehicle-api`.

Physical UAV launch
-------------------

``uav_hardware.launch.py`` starts the same manager under ``uav_<ns_id>``, but
hardcodes ``auto_launch: false``. It starts a serial Agent on ``/dev/serial0`` at
921600 baud instead of PX4 SITL/Gazebo. Optional ``enable_ulog`` starts the
installed MAVLink log-streaming script using ``/dev/ttyACM0`` and writes to
``~/px4-logs``.

Inspect its contract with:

.. code-block:: bash

   ros2 launch uav uav_hardware.launch.py --show-args

Actual serial devices, flight-controller configuration, namespace, and permissions
must match the target hardware. This command only displays arguments. Do not
assume the simulated UDP setup configures a physical PX4 serial client.

Shared launch helpers
---------------------

``vehicle_common.launch_utils.include_launch`` finds an installed package share
path and includes a Python launch file. ``check_unknown_launch_args`` warns
about undeclared names; it does not reject every typo or automatically forward
it. ``is_truthy`` recognizes ``1``, ``true``, and ``t`` after normalizing text.
``env.require_env`` raises a LaunchError for a missing required variable.
``prepend_env_path`` returns a path string with the new entry prepended; callers
must actually pass or assign that returned string.

When adding an argument, update its enum, declaration, parsing, and explicit
forwarding to child launch/node parameters. Rebuild installed launch resources
and inspect ``--show-args`` after the change. Continue to :doc:`simulation` for
what the shared simulator action does.

Sources: `UAV launch files <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/common/uav/launch>`_,
`launch helpers <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/vehicle_common/vehicle_common/launch_utils/launch_utils.py>`_.
