CLI, Debugging Tools, and Test Ground Station
=============================================

``pennair_cli`` inspects installed project resources; ``tools`` contains runnable
utilities plus a Python helper used by simulation tests. These tools support
the main runtime but are not all launched by the UAV manager.

Discover installed configuration
--------------------------------

The ``pennair`` executable dispatches subcommands through Python extension
classes. It first identifies the requested command before importing that
extension, reducing unrelated imports for help/discovery.

With the workspace sourced:

.. code-block:: bash

   ros2 run pennair_cli pennair --help
   ros2 run pennair_cli pennair mission --all
   ros2 run pennair_cli pennair mission uav
   ros2 run pennair_cli pennair mission uav basic
   ros2 run pennair_cli pennair mode ls
   ros2 run pennair_cli pennair airframe ls
   ros2 run pennair_cli pennair world --help

Mission-name listing reads installed share directories. Asking for one mission
loads/validates it. Mode listing imports the registry's discovered modules, so
it can be slower and print optional dependency errors. A resource listed by the
CLI still needs its external simulator/hardware prerequisites to run.

The current ``mode info`` parser is incomplete: it does not define/pass the
mode-ID argument expected by its handler. Use ``mode ls`` and the source registry
instead of treating that subcommand as a working lookup API.

Payload keyboard teleop
-----------------------

``keyboard_teleop`` publishes DriveCommand and ServoCommand for a named ROS
payload at 10 Hz. It belongs to the ROS payload path described in :doc:`../sae`,
not the root ESP project's direct driver API.

With a matching **simulation** payload/controller already running:

.. code-block:: bash

   ros2 run tools keyboard_teleop --ros-args -p payload_name:=payload_0

The required name determines ``/payload_0/cmd_drive`` and ``/payload_0/servo``.
W/S request forward/reverse; A/D turn; other ordinary keys request zero motion.
M/N pairs adjust linear/angular command scales, brackets adjust servo angle,
and Ctrl+C exits. Inspect topic endpoints before using it so the named
controller is the intended simulation rather than another active robot.

Teleop provides commands; it does not spawn a model or create a controller.
Its servo values and command units must match the receiving implementation.
See the `tools README <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/tools/README.md>`_
for the full key mapping and parameters.

Inspect a camera with AprilTag debug
------------------------------------

``apriltag_debug`` is a standalone Python/OpenCV visualization utility, separate
from the C++ vision plugin manager in :doc:`vision`. It subscribes to raw or
compressed images and opens a local GUI with tag outlines, IDs, distances, and
pose axes.

.. code-block:: bash

   ros2 run tools apriltag_debug --ros-args \
     -p topic:=/uav_0/camera -p compressed:=false

The default tag size and camera intrinsics are examples, not a calibration for
every camera. Override ``tag_size_m``, ``camera_fx/fy/cx/cy`` with measured values
before interpreting metric pose. Run the GUI where a display is available;
match domain/network settings if the camera publisher is on another machine.

The headless ground station in tests
------------------------------------

``tools/headless_ground_station`` is an importable Python module, not an installed
``ros2 run`` executable. ``HeadlessGroundStation`` connects over MAVLink (default
``udpin:0.0.0.0:14550``), waits for PX4 heartbeat, sends GCS heartbeats on a background
thread, and requests/observes status used by simulation assertions.

The basic UAV simulation test uses it to wait for arming, airborne state, and
landing before a deadline. It provides a ground-station peer without opening
QGroundControl. The helper and the XRCE Agent serve different connections:
MAVLink ground-station status is not the same stream as ROS ``fmu/...`` topics.
Context-manager cleanup stops its heartbeat thread and closes the connection.

For a simulation failure, ask which boundary failed: no MAVLink heartbeat,
no ROS telemetry, no mission start, or no completion before timeout. Run the
separate simulation test collection in :doc:`../../ci/checks`; avoid manually
starting a competing process on the helper's ports.

A compact debugging order
-------------------------

1. Verify the installed package/resource with ``ros2 pkg prefix --share`` and CLI.
2. Inspect launch arguments before starting programs.
3. Inspect nodes, topic types/endpoints/QoS, and services.
4. Read manager readiness/activation logs and the relevant PX4 status.
5. Record selected messages with ``ros2 bag record`` when a failure is intermittent.
6. Match the failing boundary to :doc:`mission-loading`, :doc:`uav-runtime`,
   :doc:`vehicle-api`, :doc:`simulation`, or :doc:`vision`.

Sources: `CLI dispatcher <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_cli/pennair_cli/cli.py>`_,
`tools entry points <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/tools/setup.py>`_,
`ground station helper <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/tools/headless_ground_station/ground_station.py>`_.
