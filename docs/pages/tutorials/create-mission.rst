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

Reading an Existing Mission
```````````````````````````

Before writing your own mission, read an example one to see how it's structured.
Here is ``controls/sae_2025_ws/src/uav/missions/hover.yaml``, a
mission that takes off, flies to two waypoints, and then lands:

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
            - [[0, 0, -3], 5, LOCAL]
            - [[0, 0, -6], 5, LOCAL]
        transitions:
          complete: land

      land:
        mode: uav.LandingMode

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
so you cannot use them as labels for your transitions.

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
Use the below template, adding in the FlyToPointMode call and parameters:

.. code-block:: yaml

    modes:
      start:
        mode: uav.vtol.TakeoffMode
        params:
          takeoff_type: vertical
        transitions:
          complete: <state-name>

      <state-name>:
        mode: uav.FlyToPointMode
        params:
          target: [x, y, z]
          margin: m
        transitions:
          complete: land

      land:
        mode: uav.LandingMode

Replace ``<state-name>`` with a label of your choice, ``[x, y, z]`` with a 3-tuple of coordinates in meters, and ``m`` with a margin value in meters.

.. note::

    To see what parameters a mode accepts, read its params model next to the mode itself. For example, you had ``FlyToPointMode`` declare
    ``FlyToPointParams`` at the top of ``uav/uav/modes/FlyToPointMode.py`` in the mode tutorial, which provides the schema the ``params:`` block is checked against.

Validating Your Mission
```````````````````````

To check whether your mission is well-formed without launching it, load it through the same loader
launch uses, but make sure to replace ``<mission-name>`` with the name of your mission file:

.. code-block:: bash
    :caption: Bash

    source /opt/ros/jazzy/setup.bash
    cd controls/sae_2025_ws
    source install/setup.bash
    python3 -c "from vehicle_common.runtime.mission_loader import RuntimeMission; m = RuntimeMission.load_from_path('src/uav/missions/<mission-name>.yaml'); print(list(m.modes))"

A well-formed mission prints its state names, for example ``['start', '<state-name>', 'land']``.

If the schema or params are invalid, a ``ValidationError`` is raised.
Passing in an unknown mode id raises a ``KeyError`` that lists the registered modes.
The transition labels are not checked, so a mission with a transition to a non-existent state will still load, but will fail once the mission is running.

Running Your Mission in Sim
````````````````````````````

Missions are installed into the package share directory, so build the workspace before launching:

.. code-block:: bash
    :caption: Bash

    # From monorepo root
    cd controls/sae_2025_ws
    colcon build --packages-select uav
    source install/setup.bash

Then launch SITL with your mission selected by name, again making sure to replace ``<mission-name>`` with the name of your mission file:

.. code-block:: bash
    :caption: Bash

    ros2 launch uav uav_sitl.launch.py world:=custom mission:=<mission-name>

Committing and Pushing Your Changes
```````````````````````````````````

If this was a mission meant for the main repository, follow the steps below to commit and push your changes:

.. code-block:: bash
    :caption: Bash

    git add controls/sae_2025_ws/src/uav/missions/<mission-name>.yaml

If you had not committed your new mode yet, you could add it as well:

.. code-block:: bash
    :caption: Bash

    git add controls/sae_2025_ws/src/uav/uav/modes/<mode-name>.py

Then commit your changes, adjusting the message accordingly to any other edits you made on the branch:

.. code-block:: bash
    :caption: Bash

    git commit -m "feat: add <mission-name> mission"

Finally, push your branch to GitHub:

.. code-block:: bash
    :caption: Bash

    git push -u origin user/<github-username>/add-mission

After pushing, open a pull request on GitHub from your branch into ``main``.