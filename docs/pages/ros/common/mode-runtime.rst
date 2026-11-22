:doc-priority: key
:key-reason: Read before adding behaviors: lifecycle, transitions, timing, and cleanup live here.

Mode Lifecycle and the Shared Manager
=====================================

``vehicle_common.ModeManager`` supplies the shared execution machinery.
``UAVModeManager`` and ``PayloadModeManager`` supply vehicle-specific setup and
update policies. A mode is a Python object in the manager node, not a separate
node or process. Read :doc:`mission-loading` first.

Who owns what
-------------

The manager owns the ROS timer, vehicle object, mode instances, transition maps,
and active local name. A mode holds references to that manager node, the shared
vehicle, and its validated parameters. It decides one behavior's next action
and reports an outcome; it does not choose the entire mission sequence.

``setup_modes`` walks all mission entries, constructs their registered classes
with no constructor arguments, calls ``initialize(node, vehicle, params)``, and
stores them under their local names. **All modes initialize at mission load**,
including modes that will not run until later.

Lifecycle in order
------------------

.. mermaid::

   sequenceDiagram
       accTitle: One mode transition
       participant M as Mode manager
       participant A as Current mode
       participant B as Next mode
       M->>A: initialize(node, vehicle, params) at load
       M->>B: initialize(node, vehicle, params) at load
       M->>A: activate → on_enter
       loop While current mode is active
           M->>A: update(dt) → on_update(dt)
           M->>A: check_status()
           A-->>M: continue / custom outcome
       end
       M->>A: deactivate → on_exit
       M->>B: activate → on_enter

* ``initialize`` sets references and persistent configuration. Do not command a
  takeoff here: this also runs for inactive modes.
* ``on_enter`` begins a behavior and resets state that should restart each time
  the mission enters it.
* ``on_update(dt)`` performs one bounded step. Return control so telemetry and
  other executor callbacks can run.
* ``check_status`` reports an outcome after that update.
* ``on_exit`` releases behavior-specific resources or stops temporary work.

The base ``Mode.update`` calls ``on_update`` only while its ``active`` flag is
true. ``activate`` and ``deactivate`` set that flag and call the entry/exit hooks.
Instances are reused when a mission returns to a local mode, so data stored in
``initialize`` is not automatically reset. Several existing modes initialize
counters only once; inspect them before adding loops to a mission graph.

Start and update behavior
-------------------------

The base manager loads parameters through the subclass's ``load_params`` and
checks that they provide ``vehicle_name`` and ``auto_launch``. It creates the
relative ``mode_manager/start_mission`` Trigger service. With auto-launch enabled,
a 0.1-second timer polls the subclass's readiness function.

``start_mission`` cancels readiness polling, resets the elapsed-time reference,
and creates the 0.1-second ``spin_once`` timer. Calling it again returns false
and reports that the mission already started. It starts execution; it does not
reload the YAML or create new mode instances.

``spin_once`` belongs to the subclass. In the UAV subclass it selects ``start``
and handles arming/failsafes before calling ``run_active_mode``. That shared
method calculates ``dt = time() - last_update_time``, calls the active mode's
update, then reads its status. Timing uses wall-clock ``time()`` here, not the
Gazebo clock. Read :doc:`simulation` before interpreting paused simulations.

Outcomes and transitions
------------------------

.. list-table:: Handling in the shared base manager
   :header-rows: 1

   * - Outcome
     - Behavior
   * - ``continue``
     - Keep the active mode.
   * - Custom label, e.g. ``complete``
     - Look up ``transitions[active_name][label]`` and switch to that local mode.
   * - Mapping target ``terminate``
     - Dispatch the manager's termination handler.
   * - ``error`` or ``terminate``
     - Base implementation clears its marker, deactivates the mode, and calls vehicle ``stop``.

The base implementation does not cancel its execution timer or destroy the node
in those stop handlers. The UAV subclass overrides them: ``error`` sets a
failsafe flag; ``terminate`` destroys the node. Those overrides do not call the
base cleanup in the same way. Do not rely on ``on_exit`` being called for every
UAV failure/termination path; see :doc:`uav-runtime` for the actual branches.

``run_active_mode`` catches exceptions from ``mode.update`` and dispatches
``error``. ``check_status`` and transition lookup are outside that try block.
Missing transition keys or exceptions from status checks are therefore not
covered by that handler. Registered transition labels describe intended
outcomes; the manager does not validate every graph edge at startup.

Mission-start marker
--------------------

The manager writes a small ``mission-started`` file when its loop starts and
clears it during construction. Location precedence is:

1. ``PENNAIR_MISSION_STARTED_MARKER_PATH`` if explicitly set;
2. ``DEPLOY_ROOT/state/mission-started`` if a deployment root is set;
3. ``/tmp/pennair/<vehicle_name>/mission-started`` when a name is available;
4. ``/tmp/pennair/mission-started`` otherwise.

The file is a startup signal for surrounding tooling, not a live health check.
Some subclass exit paths do not clear it. Use ROS/PX4 status to determine what
is running now.

Change a behavior without breaking its neighbors
------------------------------------------------

Decide its parameters, accepted vehicle type, completion outcomes, and entry/exit
needs before coding. Keep reusable sequencing in YAML and device-specific
commands in the vehicle adapter. Test first entry, re-entry, missing telemetry,
and failure outcomes. A mode that publishes once and then waits forever may
break a control path that expects ongoing setpoints.

Next: :doc:`uav-runtime` for the UAV subclass, or
:doc:`../../tutorials/create-first-mode` for an implementation exercise.

Sources: `mode.py <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/vehicle_common/vehicle_common/mode.py>`_
and `mode_manager.py <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/vehicle_common/vehicle_common/mode_manager.py>`_.
