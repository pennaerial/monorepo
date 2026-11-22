Shared Flight Behaviors: A Worked Mission
=========================================

This page traces the shared UAV modes using the bundled ``basic.yaml``. Read
:doc:`mode-runtime` for lifecycle and :doc:`vehicle-api` for the commands they
send. Mission modes describe behavior; PX4 modes describe autopilot state.
They are related but separate state machines.

The basic mission
-----------------

.. mermaid::

   flowchart LR
       accTitle: Bundled basic UAV mission
       accDescr: VerticalTakeoffMode completes after offboard entry, NavGPSMode completes its waypoint and dwell, and LandingMode finishes after touchdown and disarming.
       start["start · VerticalTakeoffMode"] -->|complete| gps["GPS · NavGPSMode"]
       gps -->|complete| land["land · LandingMode"]
       land -->|terminate| finished["Manager exits"]

``uav/missions/basic.yaml`` requests a 5 m vertical takeoff using PX4_AUTO,
then a ``LOCAL`` waypoint ``[5,0,-5]`` with a 1-second dwell, then landing.
The navigation limits in that file are 3 m/s horizontal, 3 m/s vertical, and
0.5 rad/s heading rate. Verify the installed copy when experimenting:

.. code-block:: bash

   ros2 run pennair_cli pennair mission uav basic
   ros2 pkg prefix --share uav

VerticalTakeoffMode
-------------------

Parameters are ``takeoff_height`` (default 5 m) and ``takeoff_method`` (default
``PX4_AUTO``). Initialization builds the offboard target ``(0,0,-abs(height))``
and resets timing/command flags.

With **PX4_AUTO**:

1. Wait for local/global position, then send one ``vehicle.takeoff(height)`` request.
2. Let PX4 own AUTO_TAKEOFF; later wait at least 1 second and for AUTO_LOITER.
3. Request OFFBOARD.
4. Return ``complete`` only after reported navigation state is OFFBOARD.

That completion condition is a state observation, not a direct height tolerance.
The takeoff helper and mode both suppress repeat commands after their first
attempt. If PX4 rejects the command, inspect status rather than expecting the
mode to retry indefinitely.

With **OFFBOARD**, each update publishes the fixed local target and periodically
requests offboard entry. It also contains an arming retry branch, though the
normal manager already handles disarming before running the mode. Completion
uses distance to that local target below 0.1 m. The fixed XY target is zero in
the local frame; it is not automatically the vehicle's current XY.

The mode's counters are initialized once and it has no custom reset-on-enter
hook. Reusing it in a looping mission needs an explicit review of that state.

NavGPSMode
----------

Each coordinate entry has the form ``[[x,y,z], dwell_seconds, LOCAL]`` or
``[[latitude,longitude,altitude], dwell_seconds, GPS]``. Local values are used
directly; GPS values are converted using the adapter's stored reference.

On each update:

1. Reject fixed-wing VTOL operation by setting a navigation error. The current
   GotoSetpoint-based mode expects multicopter flight.
2. Measure distance to the active goal, if one has been selected.
3. Publish the active GotoSetpoint with limits, or temporarily hold current
   position before the first goal is selected.
4. When there is no goal, select the first one. For a reached goal, decrement
   its dwell only while within ``margin``; otherwise continue approaching it.
5. After the dwell expires, select the next coordinate. Once the index passes
   the final coordinate, ``check_status`` returns ``complete``.

The default margin is 1 m. A positive dwell is time spent inside that region,
not simply time since the mode began. Setpoints continue while dwelling and yaw
is locked during that wait. Moving out of the margin pauses dwell countdown.
The code selects a new goal after that tick's publication, so its first outgoing
goal command normally appears on the following update.

This mode follows a list of goals. It does not invoke the common A* planner or
perform obstacle avoidance. An error returns ``error`` to the UAV manager's
failsafe handling; see :doc:`uav-runtime`.

LandingMode
-----------

Before PX4 reports AUTO_LAND, the mode publishes a current-position hold and
requests landing. Once AUTO_LAND is active, it stops sending those position
setpoints so PX4 owns the landing setpoint stream and land detector.

``check_status`` returns ``terminate`` only when the aircraft is disarmed and
has exited AUTO_LAND. The manager also has an early landing/disarmed exit branch
before the mode update. A successful landing therefore includes PX4 state
transitions; reaching a low height alone is not the completion check.

VTOL-specific behaviors
-----------------------

``uav.vtol.TransitionMode`` takes ``to_mode: MC`` or ``FW``. It waits for VTOL
status, requests a transition once, and continues to publish position references.
For FW it places a goal 10 m ahead along current yaw; for MC it holds position.
It completes when the adapter's reported ``vehicle_type`` matches the target.
During a transition the adapter retains the last stable MC/FW classification.

``uav.vtol.TakeoffMode`` supports ``vertical`` and ``horizontal`` takeoff. The
horizontal path waits for fixed-wing state and performs a staged
force-disarm → NAV_TAKEOFF → re-arm sequence with blocking sleeps. Its eventual
completion is OFFBOARD entry. This is aircraft/PX4-specific code: do not treat it
as a general-purpose hardware takeoff procedure or copy its sleeps into new modes.

Both VTOL modes currently register against the generic ``UAV`` target, so the
mission loader does not prevent selecting an unsuitable multicopter airframe.
Use the airframe and mode together intentionally. See :doc:`launch` for selection.

Try a change and observe it
---------------------------

Use the simulation in :doc:`../getting-started` with ``auto_launch:=false``.
Copy ``basic.yaml`` to a separate filepath, change one takeoff height or waypoint,
then launch that filepath and start the mission through its service. Watch
activation/outcome logs together with local-position and vehicle-status topics.
Do not change unrelated gains or frames at the same time.

For a new behavior, add a parameter schema and registration, implement bounded
updates, define completion/error outcomes, wire transitions, and check first entry
and re-entry. :doc:`workspace` covers build/test commands.

Sources: `basic mission <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/uav/missions/basic.yaml>`_,
`shared modes <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/common/uav/uav/modes>`_.
