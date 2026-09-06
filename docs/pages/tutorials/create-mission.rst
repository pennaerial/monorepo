Creating A Mission
===================

.. rst-class:: lead

    A guide to writing your own mission. You will learn how to read a mission YAML finite-state
    machine, write a new one incorporating the mode you just created ``uav.FlyToPointMode``, and fly it in the sim.

----

Before You Begin
````````````````````

This tutorial assumes that you have already cloned monorepo, are checked out to its most up-to-date main
branch, and have built the ROS workspace. If you have not, follow the :doc:`Ubuntu installation guide <../installation/ubuntu>` first.

This tutorial also assumes that you have already created a mode with the previous tutorial.
If you have not, follow the :doc:`Creating Your First Mode <create-first-mode>` tutorial first.

You should also be familiar with the following concepts:

* :doc:`Modes <../concepts/modes>`
* :doc:`Missions <../concepts/missions>`
* :doc:`Mode Manager <../concepts/mode-manager>`

Those pages explain the mode lifecycle and how transition labels connect modes
inside a mission. Here, you will put those ideas into practice.

Reading an Existing Mission
```````````````````````````

Before writing your own mission, read an example one to see how it's structured.
Here is ``controls/sae_2025_ws/src/uav/missions/basic.yaml``, a
mission that takes off, flies to a waypoint, and then lands:

.. code-block:: yaml

    modes:
      start:
        mode: uav.vtol.TakeoffMode
        params:
          takeoff_type: vertical
        transitions:
          complete: GPS

      GPS:
        mode: uav.NavGPSMode
        params:
          coordinates:
            - [[5, 0, -5], 1, LOCAL]
        transitions:
          complete: land

      land:
        mode: uav.LandingMode

.. note::

    The coordinates parameter for uav.NavGPSMode is a list of waypoints, 
    where each waypoint is a 3-D tuple of either (x, y, z) or (lat, lon, alt) coordinates depending on the frame,
    a wait time in seconds, and a frame type of either LOCAL or GPS.
    In this case, the UAV will fly to the point (5, 0, -5) with a waittime of 1 second in the LOCAL frame.

The keys nested directly under ``modes:`` are **state names**: ``start``, ``GPS``, and ``land``. These are just labels
chosen for the mission. There is no requirement that the state named ``GPS`` has to run a mode with "GPS" in its name.

Each state takes up to three keys:

.. code-block:: text

    mode          the registered id of the mode this state runs
    params        values validated against that mode's params model
    transitions   a map from a check_status() label to the next state's name

When a mission begins, ``UAVModeManager`` switches to the state named ``start`` first,
so a mission without this state never begins.

While ordering inside the file doesn't technically matter,
it is good to keep them organized in a logically clear manner.

Read the transitions as ``label: target``. For example, ``GPS`` has ``complete: land``, which means that when this mode reports ``complete``,
the program should go to the state named ``land``. Note that the target is a **state name in this file**, not an actual mode id.

The manager checks for three reserved strings via ``check_status()`` before it ever looks at your ``transitions:`` block,
so you cannot use them as labels for your transitions, read more at :doc:`Modes <../concepts/modes>`.

.. note::

    The ``land`` state has no ``transitions`` block at all, since it is a terminal state, as ``LandingMode``
    returns the reserved ``terminate`` string.

Creating a Branch
`````````````````

Before making any changes, create a new branch for your mission (or stay on the same one you used to create the new mode).
Use the following naming convention, replacing ``<github-username>`` with your GitHub username:

.. code-block:: bash
    :caption: Bash

    git checkout -b user/<github-username>/add-mission

Writing Your Mission
````````````````````

Create a new YAML file in the UAV missions directory and name it accordingly:

.. code-block:: text

    monorepo/controls/sae_2025_ws/src/uav/missions/<mission-name>.yaml

You should attempt to create a mission that takes off, calls the mode you created in the previous tutorial ``uav.FlyToPointMode``,
passing in a required 3-tuple target argument and optionally a margin argument, then lands.
Use the above ``basic.yaml`` as a  template, replacing ``uav.NavGPSMode`` with the ``uav.FlyToPointMode`` call and parameters:

.. note::

    To see what parameters a mode accepts, read its params model next to the mode itself. For example, you had ``FlyToPointMode`` declare
    ``FlyToPointParams`` at the top of ``controls/sae_2025_ws/src/uav/uav/modes/fly_to_point_mode.py`` in the mode tutorial,
    which provides the schema the ``params:`` block is checked against.

Here is a completed version of the mission:

.. dropdown:: Complete Mission fly_to_point.yaml

    .. code-block:: yaml

        modes:
          start:
            mode: uav.vtol.TakeoffMode
            params:
              takeoff_type: vertical
            transitions:
              complete: fly_to_point

          fly_to_point:
            mode: uav.FlyToPointMode
            params:
              target: [5.0, 0.0, -5.0]
              margin: 1.0
            transitions:
              complete: land

          land:
            mode: uav.LandingMode

Running Your Mission in Sim
````````````````````````````

Missions are installed into the package share directory, so build the workspace before launching:

.. code-block:: bash
    :caption: Bash

    # From monorepo root
    cd controls/sae_2025_ws
    colcon build --packages-select uav
    source install/setup.bash

Then launch SITL with your mission selected by name, making sure to replace ``<mission-name>`` with the name of your mission file:

.. code-block:: bash
    :caption: Bash

    ros2 launch uav uav_sitl.launch.py mission:=<mission-name>
