Getting Started with ROS
========================

Goal: build the workspace, inspect its running nodes, and follow one simulated
UAV mission. Use :doc:`common` to find the implementation afterward. These
commands use Bash, Ubuntu 24.04, and ROS 2 Jazzy; complete
:doc:`software setup <../installation/index>` first.

The pieces you will use
-----------------------

A **package** groups code, dependencies, and installed resources. A **node** is a
running ROS participant, usually responsible for one job. Nodes exchange typed
messages through **topics**, request a response through **services**, and receive
configuration through **parameters**. ROS **actions** support longer operations
with feedback and cancellation; our mission manager currently uses its own
mode lifecycle and a start service. A **launch file** starts a collection of
programs with their arguments and names.

A **mission** is our YAML plan of behaviors and transitions. Each **mode** is a
Python behavior class. The **mode manager** is a ROS node that runs one mode at
a time. Modes are objects inside that node; they are not separate ROS nodes.
PX4 also has flight modes such as OFFBOARD and AUTO_LAND; those are autopilot
states, distinct from our mission's behavior classes.

.. mermaid::

   flowchart LR
       accTitle: ROS mission execution
       accDescr: Launch and mission configuration create a manager. Its active mode uses a vehicle adapter to exchange commands and telemetry with PX4 through an Agent.
       launch["Launch file"] --> manager["Mode manager node"]
       yaml["Mission YAML"] --> manager
       manager --> behavior["Active mode object"]
       behavior --> vehicle["Vehicle adapter"]
       vehicle -->|"commands / setpoints"| agent["XRCE Agent ↔ PX4"]
       agent -->|"telemetry"| vehicle
       px4["PX4 flight controller"] <--> agent
       px4 <--> environment["Gazebo simulation / physical UAV"]

For simulation, Gazebo generates motion and sensors while PX4 runs the flight
controller. ROS supplies higher-level behavior. For physical flight, the
simulated vehicle is replaced by hardware; the simulation command below is for
the computer setup.

Build and source
----------------

From the repository root, after installing ROS and project dependencies:

.. code-block:: bash

   source dev_env.sh
   source /opt/ros/jazzy/setup.bash
   cd ros_ws
   rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
   colcon list
   colcon build --symlink-install
   source install/setup.bash

``rosdep`` installs declared system dependencies; it does not replace the
project's additional Python dependencies or PX4/Gazebo setup. Mission loading
requires Pydantic 2. Python mode sources use Python 3.12 syntax, matching Ubuntu
24.04's interpreter. Use the environment prepared by the installation guide.

``build/`` holds intermediate files; ``install/`` holds the packages ROS actually
finds; ``log/`` holds build logs. Sourcing ROS provides the base tools, and
sourcing ``install/setup.bash`` overlays this workspace. ``dev_env.sh`` sets
repository paths used by launch files; it does not source ROS for you.

In each new terminal, from the repository root:

.. code-block:: bash

   source dev_env.sh
   source /opt/ros/jazzy/setup.bash
   source ros_ws/install/setup.bash

For later edits, from ``ros_ws/``, ``colcon build --packages-up-to uav`` builds
``uav`` and its declared workspace dependencies. ``--packages-select uav`` builds
only that package and assumes dependencies are already available. Launch-time
packages are not always all declared by the target, so use a full first build.
Rebuild after adding entry points, interfaces, or installed YAML/launch resources,
then source the overlay again. Symlink installation does not eliminate these steps.

Explore before launching
------------------------

.. code-block:: bash

   ros2 pkg list
   ros2 pkg executables uav
   ros2 pkg prefix --share uav
   ros2 run pennair_cli pennair mission uav
   ros2 run pennair_cli pennair mission uav basic
   ros2 run pennair_cli pennair mode ls
   ros2 launch uav uav_sitl.launch.py --show-args

``ros2 run`` selects one installed executable. ``ros2 launch`` reads a launch
file; ``name:=value`` arguments configure that launch. The PennAiR CLI lists
installed missions and registered modes. Mode discovery imports optional project
modules, so an import error can indicate a missing dependency rather than an
empty mode registry. ``--show-args`` inspects the declared launch options without
starting the vehicle stack.

Run one simulated UAV
---------------------

Prerequisites beyond the workspace: built PX4 SITL, Gazebo Harmonic, the
``gz-models`` submodule/resources, and ``MicroXRCEAgent`` on PATH. PX4's build
command, from the repository root after its host dependencies are installed, is:

.. code-block:: bash

   make -C PX4-Autopilot px4_sitl

Start QGroundControl for the simulated vehicle's ground-station connection.
Then, in a prepared ROS terminal:

.. code-block:: bash

   ros2 launch uav uav_sitl.launch.py \
     mission_package:=uav mission:=basic airframe:=quadcopter \
     world:=default ns_id:=0 auto_launch:=false

The default standalone launch starts Gazebo through ``sim2.launch.py``, one PX4
instance, the Agent on UDP port **8888**, and ``/uav_0/mode_manager``. The root
ESP project's example uses Agent port **7777**; these are separate client setups.
``auto_launch:=false`` leaves mission execution idle while the processes and
telemetry start. It does not prevent those processes from launching.

In a second prepared terminal, inspect the system before starting the mission:

.. code-block:: bash

   ros2 node list
   ros2 topic list -t
   ros2 service list -t
   ros2 node info /uav_0/mode_manager
   ros2 param list /uav_0/mode_manager

When the simulated vehicle has valid telemetry and preflight conditions, start
its mission:

.. code-block:: bash

   ros2 service call /uav_0/mode_manager/start_mission std_srvs/srv/Trigger '{}'

The bundled ``basic`` mission takes off to 5 m, navigates to a local waypoint,
and lands. Watch the mode-manager logs for activation and transition messages.
The manual service starts the update loop; it is not a substitute for inspecting
PX4 readiness. With the default ``auto_launch:=true``, the UAV manager waits for
its readiness checks before starting the loop automatically. Add ``headless:=true``
to the single-UAV launch to omit the Gazebo GUI.

Use ``Ctrl+C`` in the launch terminal to end the simulation session. Avoid
starting another standalone launch against the same ports while it is running.

A few commands for debugging
----------------------------

.. code-block:: bash

   ros2 topic info /uav_0/fmu/out/vehicle_local_position_v1 --verbose
   ros2 topic echo /uav_0/fmu/out/vehicle_local_position_v1 \
     --qos-reliability best_effort --qos-durability volatile --once
   ros2 interface show px4_msgs/msg/VehicleLocalPosition
   ros2 param get /uav_0/mode_manager auto_launch
   ros2 service type /uav_0/mode_manager/start_mission
   ros2 action list

These are the topic names used by the current UAV adapter. Check ``topic list``
first if you use a different namespace or PX4 version. Quality of service (QoS)
controls delivery compatibility; a default reliable subscriber cannot receive
from a best-effort publisher. A topic being listed does not prove messages are
arriving. Check its endpoints, message type, and matching QoS.

Names starting with ``/`` are absolute. Relative names are resolved under the
node's namespace, so ``fmu/out/...`` becomes ``/uav_0/fmu/out/...`` for this launch.
Keep ``ROS_DOMAIN_ID`` consistent between terminals and machines that should
communicate. It groups ROS discovery; it is not the Agent's UDP port.

Launch arguments such as ``world:=default`` configure the launch file. Node
parameters such as ``auto_launch`` belong to a running node; they are related
only where the launch file explicitly passes a value into that node. A declared
parameter is not necessarily safe or implemented to change while running.

To save telemetry for later inspection, record a bag in a writable directory:

.. code-block:: bash

   ros2 bag record -o uav_debug /uav_0/fmu/out/vehicle_local_position_v1
   # Stop recording with Ctrl+C, then inspect the saved metadata:
   ros2 bag info uav_debug

Choose a new output name for each recording. Recording a selected topic captures
its messages; it does not save the entire simulation state or mission configuration.

Missions and modes: your next step
----------------------------------

Read :doc:`../concepts/missions`, :doc:`../concepts/modes`, and
:doc:`../concepts/mode-manager` for the behavior model. Then follow
:doc:`../tutorials/create-first-mode` and :doc:`../tutorials/create-mission`.
Use the installed ``basic`` mission and current mode registry for executable
examples; abbreviated conceptual examples omit required parameters.

For source ownership, use :doc:`common`. Competition-specific work lives in
:doc:`iarc` and :doc:`sae`. Automatic checks are covered in :doc:`../ci/checks`.

If something fails
------------------

* **Package/executable missing:** check the build summary, ``ros2 pkg prefix``,
  and which workspace you sourced. Rebuild newly installed resources.
* **Missing PENNAIR path:** source ``dev_env.sh`` before evaluating launch files.
* **Mission or mode validation error:** inspect the named YAML, registered mode
  ID, parameter schema, and optional-module import errors. Pydantic errors show
  the field that failed.
* **No telemetry:** inspect Agent startup/port 8888 and PX4 logs; compare topic
  names and QoS before changing mission logic.
* **Mission idle:** with auto-launch disabled, call the start service. With it
  enabled, inspect PX4 telemetry and preflight readiness logs.
* **Wrong motion direction:** confirm the coordinate frame. PX4 local NED uses
  positive Z down, so a point above the origin has negative Z.

Source: `single-UAV launch <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/uav/launch/uav_sitl.launch.py>`_.
Further learning: `ROS 2 Jazzy CLI tutorials <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html>`_
and `PX4 ROS 2 guide <https://docs.px4.io/main/en/ros2/user_guide.html>`_.
