:doc-status: evolving
:status-reason: SAE mission wiring, payload control, perception, and competition interfaces are changing. This page describes ownership and current structure.

ROS SAE: Project Structure
==========================

``ros_ws/src/sae/`` groups the ground-payload behavior, its ROS motor-control
backend, path following, and project interfaces. SAE also uses shared UAV,
mission, simulation, and vision packages under :doc:`common`.

How the systems interact
------------------------

.. mermaid::

   flowchart TB
       accTitle: SAE payload command path
       accDescr: The payload manager runs a behavior using the Python vehicle adapter. ROS drive and servo commands reach a simulation or GPIO backend.
       mission["Payload mission YAML"] --> manager["PayloadModeManager"]
       manager --> modes["Active payload behavior"]
       modes --> adapter["Python Payload vehicle adapter"]
       adapter -->|"cmd_drive / servo"| control["ROS payload_controller"]
       control --> sim["SimController → Gazebo"]
       control --> hardware["GPIOController → Pi motors / servo"]
       control -. "motor feedback" .-> consumers["Telemetry consumers"]
       planner["Path follower / planning integration"] -. "integration varies" .-> modes

This is the general command path. Planner/perception use depends on the selected
mission; the diagram does not claim every package is connected in every launch.

.. important::

   ``ros_ws/src/sae/payload_controller`` is a ROS C++ package for simulation and
   Raspberry Pi GPIO control. The repository-root ``payload_controller/`` is a
   separate ESP-IDF project with its own Linux/ESP32-S3 builds and DDS client.
   Its detailed guide is :doc:`../pc/index`. Commands, configurations, and
   hardware assumptions from one project do not automatically apply to the other.

Package map
-----------

.. list-table:: Paths relative to ``ros_ws/src/sae/``
   :header-rows: 1
   :widths: 24 46 30

   * - Package
     - Role
     - First files to read
   * - ``payload``
     - Python vehicle adapter, payload mission manager, behaviors, mission resources, SITL launch.
     - ``payload/PayloadModeManager.py``, ``payload/payload.py``, ``launch/payload_sitl.launch.py``.
   * - ``payload_controller``
     - C++ ROS node and selectable simulation/GPIO implementations, motor/encoder/servo code.
     - ``controller.hpp``, ``payload_controller.cpp``, ``plugins.xml``, parameter schema and launch file.
   * - ``payload_interfaces``
     - Drive/servo commands, motor state/heartbeat, timed-drive and tuning-related services.
     - ``msg/`` and ``srv/``.
   * - ``uav_interfaces``
     - Project UAV/perception request types shared by packages elsewhere too.
     - ``srv/`` and package dependency declarations.
   * - ``path_follower``
     - C++ path-following implementation.
     - ``path_follower.hpp`` and ``src/path_follower.cpp``; trace actual callers before assuming mission use.

The Python Payload adapter publishes relative ``cmd_drive`` and ``servo`` topics
and creates service clients such as timed driving. Launch namespaces turn these
into vehicle-specific names, for example ``/payload_0/cmd_drive``. The ROS
controller subscribes to those commands and delegates to its selected backend.
Its parameter file defaults to ``GPIOController``; simulation launch must select
the simulation backend. Inspect the launch arguments and resolved parameters
before assuming a standalone node is configured for simulation.

Where configuration lives
-------------------------

* ``payload/missions/`` holds mission YAML; ``payload/modes/`` holds registered
  payload behaviors. The shared loader/registry is still ``vehicle_common``.
* ``payload/config/payload_mission_parameters.yaml`` defines mission-node parameters.
* ``payload_controller/config/payload_controller_params.schema.yaml`` defines
  controller parameters; the accompanying values file includes pins, wheel
  geometry, encoder calibration, PID gains, servo settings, and simulator world.
* ``common/sim/simulations/sae/`` and ``gz-models/worlds/sae.sdf`` cover SAE
  simulation resources. Some shared camera/vision code also lives under common.

A change to a message or service affects both ends of the connection. Rebuild
the interface package and consumers, then source the updated workspace. Pin
assignments and tuning values describe a particular physical setup; check them
against the current robot instead of treating them as universal defaults.

Start by inspecting the installed system
----------------------------------------

After the full workspace build in :doc:`getting-started`:

.. code-block:: bash

   ros2 pkg executables payload
   ros2 pkg executables payload_controller
   ros2 run pennair_cli pennair mission payload
   ros2 launch payload payload_sitl.launch.py --show-args
   ros2 launch payload_controller payload_controller.launch.py --show-args
   ros2 interface show payload_interfaces/msg/DriveCommand
   ros2 interface show payload_interfaces/msg/MotorState

These discovery commands do not start motors. Use the current launch options
and mission files to choose a simulation configuration; the project-specific
startup sequence is evolving. For a running simulation, inspect ``ros2 node list``,
``ros2 topic list -t``, and ``ros2 service list -t`` to confirm names and
connections rather than copying an old namespace.

Testing boundaries
------------------

``payload_controller/test/`` contains control-math and shutdown tests as well as
motor/servo experiments. Some binaries and scripts directly exercise GPIO and
require physical wiring plus a running pigpio daemon. They are not interchangeable
with simulation tests. Start with package test configuration and the relevant
backend source before running a device-specific experiment.

The broad map here should survive changes to individual missions. Record new
behavior in the owning package and update this map when responsibilities move.
For automatic checks and the current ARM artifact limitations, see
:doc:`../ci/checks` and :doc:`../ci/delivery`.

Sources: `SAE packages <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/sae>`_,
`payload controller <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/sae/payload_controller>`_.
