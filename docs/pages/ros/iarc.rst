:doc-status: evolving
:status-reason: IARC trajectories, controllers, mission parameters, and competition behavior change frequently. This page is a structural guide.

ROS IARC: Project Structure
===========================

``ros_ws/src/iarc/`` currently contains ``iarc_flight``: project-specific flight
behaviors, trajectories, and tracking controllers. It builds on the shared
:doc:`UAV and mission framework <common>` rather than replacing it. Read
:doc:`getting-started` before launching a competition mission.

How it connects
---------------

.. mermaid::

   flowchart LR
       accTitle: IARC behavior and tracking
       accDescr: The shared manager runs TrackMode, which combines a trajectory and measured state through a tracking controller to command PX4.
       mission["IARC mission YAML"] --> manager["Shared UAV mode manager"]
       manager --> track["iarc_flight.TrackMode"]
       trajectory["Trajectory · desired motion over time"] --> track
       state["UAV state feedback"] --> track
       track --> controller["Tracking controller"]
       controller --> adapter["UAV setpoint API → PX4"]
       adapter --> state

A trajectory answers “where should the vehicle be at this time?” A tracking
controller uses that reference and the vehicle state to choose a setpoint.
``TrackMode`` brings them together within the ordinary mode lifecycle. PX4 still
runs the underlying flight controller.

Where to look
-------------

Paths below are relative to ``ros_ws/src/iarc/iarc_flight/``:

* ``missions/``: YAML sequences using shared takeoff/landing and project modes.
  Circle, gentle, MPC, and minimum-snap examples are installed through ``setup.py``.
* ``iarc_flight/modes/track_mode.py``: ``iarc_flight.TrackMode`` and its parameter
  schema. Begin here to see which trajectory/controller settings are accepted.
* ``iarc_flight/trajectories/``: common trajectory interface plus circle and
  minimum-snap implementations. Follow the current factories/registries to see
  which names are selectable from YAML.
* ``iarc_flight/controllers/``: controller interface, feedforward, and MPC.
  Feedforward passes the planned reference toward PX4; MPC repeatedly solves a
  limited-horizon tracking problem with configured constraints.
* ``package.xml`` and ``setup.py``: package dependencies and installed resources.
  Root ``pyproject.toml`` also carries solver dependencies such as cvxpy and Drake.

Keep trajectory generation, tracking, and shared vehicle commands separate when
adding a feature. A competition-specific mode belongs here; a reusable UAV
operation may belong in ``common/uav``. Discuss that boundary before promoting
an experimental implementation into common code.

Inspect and try the current examples
------------------------------------

From ``ros_ws/`` after installing dependencies:

.. code-block:: bash

   colcon build --packages-up-to iarc_flight
   source install/setup.bash
   ros2 run pennair_cli pennair mission iarc_flight
   ros2 run pennair_cli pennair mission iarc_flight iarc_circle
   ros2 launch uav uav_sitl.launch.py --show-args

The selective build assumes shared simulation/launch packages have already been
built as in :doc:`getting-started`. Start QGroundControl, then run one simulated
vehicle while leaving the mission idle for inspection:

.. code-block:: bash

   ros2 launch uav uav_sitl.launch.py world:=iarc \
     mission_package:=iarc_flight mission:=iarc_circle auto_launch:=false

Inspect telemetry and start through the service described in
:doc:`getting-started`. The current circle mission takes off, tracks its circle,
then lands. Read the YAML before launching; parameters and solver requirements
can change. The multi-UAV wrapper accepts the same mission package/name and a
``num_uavs`` argument; each UAV has its own local origin.

Before changing algorithms
--------------------------

Trace the YAML to ``TrackMode``, its trajectory selection, controller selection,
and the final UAV setpoint call. Check units, coordinate frame, timing, completion
conditions, and solver-failure behavior at those boundaries. Mission files are
working configurations for particular experiments, not permanent interface
promises. This page intentionally leaves detailed solver mathematics to a future
reviewed guide as those implementations settle.

Sources: `IARC package <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/iarc/iarc_flight>`_
and `current examples <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/iarc/iarc_flight/missions>`_.
