ROS Common: Shared Framework and UAVs
=====================================

Common is the shared runtime in ``ros_ws/src/common``. Use this page to see how
its packages connect, then read the focused tabs below. For the first build/run,
start with :doc:`getting-started`; this section explains the implementation.

How the systems fit together
----------------------------

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

The manager owns the ROS node, timers, and active mode. A mode calls its vehicle
adapter; the adapter publishes requests and saves incoming telemetry. PX4
controls flight. Gazebo supplies the simulated physical environment; bridges
expose selected simulator data as ROS interfaces. Perception and planning are
optional connections that must be wired into a particular behavior.

Shortest useful reading paths
-----------------------------

Only three Common pages carry blue **Key** markers: mission loading, the mode
lifecycle, and the vehicle API. They explain the central contracts shared by
other subsystems. The Common section itself and specialized guides are not
marked Key. Follow the path for your task below; WIP and human-review labels
describe separate things.

* **Changing a mission or adding a mode:** :doc:`common/mission-loading` →
  :doc:`common/mode-runtime` → :doc:`common/flight-modes`. Then use the
  :doc:`mode tutorial <../tutorials/create-first-mode>`.
* **Changing UAV commands or feedback:** :doc:`common/uav-runtime` →
  :doc:`common/vehicle-api`; use :doc:`common/flight-modes` to trace a caller.
* **Changing launch or multi-vehicle wiring:** :doc:`common/launch` →
  :doc:`common/simulation`. These explain ownership and naming rather than
  repeat the laptop-installation steps.
* **Working on perception or a planner:** read :doc:`common/vision` or
  :doc:`common/planning`, then identify the output consumer. Both remain WIP.
* **Finding files, build rules, or utilities:** :doc:`common/workspace` and
  :doc:`common/tools` are lookup guides you can return to as needed.

Mission framework
-----------------

.. toctree::
   :maxdepth: 1

   common/mission-loading
   common/mode-runtime

UAV behavior and transport
--------------------------

.. toctree::
   :maxdepth: 1

   common/uav-runtime
   common/vehicle-api
   common/flight-modes
   common/launch

Simulation and shared algorithms
--------------------------------

.. toctree::
   :maxdepth: 1

   common/simulation
   common/gazebo
   common/vision
   common/planning

Package lookup and tools
------------------------

.. toctree::
   :maxdepth: 1

   common/workspace
   common/tools

Common describes reuse, not a maturity guarantee. Vision/planner integration,
world generation, and some bridges have unfinished pieces documented on their
own pages. Competition-specific structures are in :doc:`iarc` and :doc:`sae`;
the separate embedded project is :doc:`../pc/index`.
