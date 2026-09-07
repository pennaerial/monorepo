Creating Your First Mode
========================

.. rst-class:: lead

   Create a UAV mode that flies to one local coordinate and verify that the mode can be discovered.

----

In this tutorial, you will create ``FlyToPointMode``. The mode
commands the UAV to fly toward one position and report ``"complete"`` when it
arrives. In the next tutorial, you will place this mode between takeoff and
landing to create a complete mission.


Before You Begin
----------------

This tutorial assumes that you have installed the monorepo and built the ROS
workspace. If you have not, follow the :doc:`Ubuntu installation guide
<../installation/ubuntu>` first.

You should also be familiar with the following concepts:

* :doc:`Modes <../concepts/modes>`
* :doc:`Missions <../concepts/missions>`
* :doc:`Mode Manager <../concepts/mode-manager>`

Those pages explain the mode lifecycle and how transition labels connect modes
inside a mission. Here, you will put those ideas into practice.


Decide What the Mode Should Do
------------------------------

This mode has one job: move the UAV to a single point. While it is active, it
will:

#. Read a target position and arrival margin supplied by the mission.
#. Continuously command the UAV to fly toward that target.
#. Measure how far the UAV is from the target.
#. Report ``"continue"`` while the UAV is still traveling and ``"complete"``
   once it is within the arrival margin.

The target will use the UAV's local NED coordinate frame:

* Positive X points north (N).
* Positive Y points east (E).
* Positive Z points down (D).

For example, ``[5.0, 0.0, -5.0]`` represents a point 5 meters north and 5
meters above the local origin.


Creating a Branch
`````````````````

Before making any changes, create a new branch for your mission.
Use the following naming convention, replacing ``<github-username>`` with your GitHub username:

.. code-block:: bash
    :caption: Bash

    git checkout -b user/<github-username>/add-mode

Create the Mode File
--------------------

UAV modes live under ``controls/sae_2025_ws/src/uav/uav/modes/``. From the
monorepo root, create:

.. code-block:: text

   controls/sae_2025_ws/src/uav/uav/modes/fly_to_point_mode.py

Start the file with these imports:

.. code-block:: python
   :caption: fly_to_point_mode.py

   from typing import override

   from rclpy.node import Node

   from uav.vehicles.UAV import UAV
   from vehicle_common.mode import Mode
   from vehicle_common.mode_loader import ParamsBase, register_mode

``UAV`` is the vehicle interface the mode will control. ``Mode`` supplies the
common lifecycle, while ``ParamsBase`` and ``register_mode`` define the mode's
configuration and make it discoverable.


Define the Parameters
---------------------

Below the imports, define the values that a mission can provide:

.. code-block:: python

   class FlyToPointParams(ParamsBase):
       target: tuple[float, float, float]
       margin: float = 0.5

``ParamsBase`` is a Pydantic model. When a mission is loaded, its ``params`` 
are validated and converted into a ``FlyToPointParams`` object.

``target`` has no default, so every use of this mode must provide three
coordinates. ``margin`` is optional and defaults to 0.5 meters.


Register the Mode
-----------------

Next, add the decorator and class declaration:

.. code-block:: python

   @register_mode(
       id="uav.FlyToPointMode",
       params_cls=FlyToPointParams,
       targets=[UAV],
       transition_labels=["complete"],
   )
   class FlyToPointMode(Mode[UAV, FlyToPointParams]):
       """Fly a UAV to one position in its local NED frame."""

The decorator records the information needed to use this class:

``id``
   The unique name used in mission YAML. UAV mode IDs use the ``uav.`` prefix.

``params_cls``
   The class that validates this mode's mission parameters.

``targets``
   The vehicle types that can run this mode.

``transition_labels``
   The custom outcomes this mode can report. A mission using this mode defines
   which mode should follow each outcome.


Initialize the Mode
-------------------

Add ``initialize()`` inside ``FlyToPointMode``:

.. code-block:: python

       @override
       def initialize(
           self,
           node: Node,
           vehicle: UAV,
           params: FlyToPointParams,
       ) -> None:
           self.node = node
           self.vehicle = vehicle
           self.p = params

The Mode Manager supplies these arguments when it prepares the mission. Saving
them on ``self`` gives the rest of the mode access to the ROS node, UAV, and
validated parameters. Existing modes use ``self.p`` as the conventional name
for their parameters.


Command the UAV
---------------

Add ``on_update()`` below ``initialize()``:

.. code-block:: python

       @override
       def on_update(self, time_delta: float) -> None:
           if self.vehicle.local_position is None:
               return

           self.vehicle.publish_position_setpoint(self.p.target)

The Mode Manager calls this method repeatedly while the mode is active. The
position check prevents the mode from using vehicle data before it is
available.

The position setpoint is published on every update rather than only once. PX4
offboard control requires a continuous stream of setpoints while it controls
the UAV.

``time_delta`` is the number of seconds since the previous update. This mode
does not need it, but it may be useful for modes that need to measure elapsed time.


Report When the UAV Arrives
---------------------------

Finish the class by adding ``check_status()``:

.. code-block:: python

       @override
       def check_status(self) -> str:
           if self.vehicle.local_position is None:
               return "continue"

           distance = self.vehicle.distance_to_waypoint("LOCAL", self.p.target)
           if distance <= self.p.margin:
               return "complete"

           return "continue"

The method keeps the mode active until the UAV is within ``margin`` meters of
the target. Returning ``"complete"`` reports that the behavior has finished.
In the next tutorial, the mission will connect this outcome to another mode.

At this point, your file should contain the imports, ``FlyToPointParams``, the
decorated ``FlyToPointMode`` class, and its three required lifecycle methods.


.. dropdown:: Complete fly_to_point_mode.py

   .. code-block:: python
      :caption: fly_to_point_mode.py

      from typing import override

      from rclpy.node import Node

      from uav.vehicles.UAV import UAV
      from vehicle_common.mode import Mode
      from vehicle_common.mode_loader import ParamsBase, register_mode


      class FlyToPointParams(ParamsBase):
          target: tuple[float, float, float]
          margin: float = 0.5


      @register_mode(
          id="uav.FlyToPointMode",
          params_cls=FlyToPointParams,
          targets=[UAV],
          transition_labels=["complete"],
      )
      class FlyToPointMode(Mode[UAV, FlyToPointParams]):
          """Fly a UAV to one position in its local NED frame."""

          @override
          def initialize(
              self,
              node: Node,
              vehicle: UAV,
              params: FlyToPointParams,
          ) -> None:
              self.node = node
              self.vehicle = vehicle
              self.p = params

          @override
          def on_update(self, time_delta: float) -> None:
              if self.vehicle.local_position is None:
                  return

              self.vehicle.publish_position_setpoint(self.p.target)

          @override
          def check_status(self) -> str:
              if self.vehicle.local_position is None:
                  return "continue"

              distance = self.vehicle.distance_to_waypoint("LOCAL", self.p.target)
              if distance <= self.p.margin:
                  return "complete"

              return "continue"


Build and Source the Workspace
------------------------------

Build the ``uav`` package so the installed workspace includes the new file:

.. code-block:: bash
   :caption: Bash

   # from monorepo root
   cd controls/sae_2025_ws
   colcon build --packages-select uav
   source install/setup.bash


Verify Mode Discovery
---------------------

Use the PennAiR CLI to list the registered modes:

.. code-block:: bash
   :caption: Bash

   pennair mode ls

Find this entry in the output:

.. code-block:: text

   Mode: uav.FlyToPointMode
     class:       FlyToPointMode
     params:      FlyToPointParams
     targets:     UAV
     vision:      —
     peers:       —
     camera:      No
     transitions: complete

If the entry appears, the mode was successfully registered.


How Mode Discovery Works
------------------------

Mission files refer to modes by registered IDs such as
``uav.FlyToPointMode``. The mode registry connects that ID to the Python class,
its parameter type, its supported vehicles, and its possible outcomes.

When the registry is first requested, it imports the Python modules inside the
``uav.modes`` and ``payload.modes`` packages. Importing
``fly_to_point_mode.py`` runs its ``@register_mode(...)`` decorator, which adds the
mode to the registry. You do not need to add the class to a central list or
import it from ``uav/modes/__init__.py``.

Automatic discovery succeeds when:

* The file is inside one of the discovered ``modes`` packages.
* The module imports without errors.
* The registered ID is unique.

The ``pennair mode ls`` command reads this registry. Seeing
``uav.FlyToPointMode`` in its output confirms that the file was imported and the
decorator registered the class.

Committing Your Changes
-----------------------

To save your progress locally, commit your changes to the branch you created at the start of this tutorial.

.. code-block:: bash
    :caption: Bash

    git commit -m "<Descriptive message about your changes>"

Next Step
---------

You now have a discoverable mode with configurable inputs and a ``complete``
outcome. Continue to :doc:`Creating a Mission <create-mission>` to connect it
to the existing takeoff and landing modes and test the complete sequence in
simulation.
