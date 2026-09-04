Modes
=====

.. rst-class:: lead

   A mode represents one behavior that a vehicle can perform as part of a
   mission.

----

What Is a Mode?
```````````````

A mode is a reusable unit of vehicle behavior. For example, a mode might tell a
UAV to take off, navigate to a waypoint, or land. A payload mode might wait for
a sensor, follow a target, or stop for a period of time.

Each mode is responsible for one part of a mission. The :doc:`mission <missions>` describes how
the modes are connected, while the :doc:`Mode Manager <mode-manager>` activates
the correct mode and runs it until it reports what should happen next.

For a given vehicle, only one mode is active at a time. This gives each mode
clear ownership of the vehicle while it performs its behavior.


Modes in a Mission
``````````````````

In a :doc:`mission <missions>`, each use of a mode has a local name and refers
to a reusable mode type. The mission can also provide parameters that adjust
the behavior and define which mode should follow each possible outcome.

For example:

.. code-block:: yaml

   modes:
     start:
       mode: uav.VerticalTakeoffMode
       params:
         takeoff_height: 5.0
       transitions:
         complete: navigate

In this example, ``start`` is the name of this mode within the mission, while
``uav.VerticalTakeoffMode`` identifies the reusable mode type. The
``takeoff_height`` parameter configures this particular use of the mode. When
the mode reports ``"complete"``, the mission continues to the mode named
``navigate``.

The same mode type can appear multiple times in a mission under different local
names or with different parameters.


Mode Lifecycle
``````````````

Every mode follows the same lifecycle:

.. list-table::
   :header-rows: 1

   * - Stage
     - When it runs
     - Purpose
   * - ``initialize()``
     - Once when the mission is first loaded
     - Prepare the mode and its starting state
   * - ``on_enter()``
     - Whenever the mode becomes active
     - Begin or reset the behavior
   * - ``on_update()``
     - Repeatedly while the mode is active
     - Perform the mode's behavior
   * - ``check_status()``
     - After each update
     - Report the outcome of the behavior
   * - ``on_exit()``
     - Whenever the mode is deactivated
     - Finish the behavior and leave the vehicle ready for what comes next

.. note::
    Loading a mission initializes every mode listed in the mission, but only the active mode enters the
    update and status-check cycle. When the mission moves to another mode, the
    current mode exits before the next one enters.



Mode Status and Transitions
```````````````````````````

After each update, ``check_status()`` returns a string that tells the Mode
Manager what to do:

``"continue"``
   Keep the current mode active.

``"terminate"``
   Complete the mission and stop the vehicle safely.

``"error"``
   Stop the mission and vehicle because the mode cannot safely continue.

Any other value is treated as a custom transition label, such as
``"complete"`` or ``"target_lost"``, and is used to determine
which mode should run next.

This separation keeps modes reusable: a mode focuses on one behavior and
reports its outcome, while the mission decides what follows.

For a guided example of creating a mode, see
:doc:`Creating Your First Mode <../tutorials/create-first-mode>`.
