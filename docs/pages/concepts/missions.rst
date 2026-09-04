Missions
========

.. rst-class:: lead

   A mission describes the behaviors a vehicle follows to accomplish a goal.

----

What Is a Mission?
``````````````````

A mission is the overall plan for a vehicle. It combines individual
:doc:`modes <modes>` into a larger behavior, such as taking off, navigating to
a destination, delivering a payload, and landing.

Each mode is responsible for one behavior. The mission is responsible for
connecting those behaviors and deciding what should happen after each one.
This lets the same mode participate in many different missions without knowing
what came before it or what should come after it.


How a Mission Progresses
````````````````````````

A mission begins with a starting mode. While that mode is active, it performs
its behavior and reports an outcome. The mission uses that outcome to select
the next mode. This process repeats until the mission is complete or cannot
continue safely.

A simplified mission might look like this:

.. code-block:: yaml
   :caption: yaml

   modes:
     start:
       mode: uav.vtol.TakeoffMode
       transitions:
         complete: navigate

     navigate:
       mode: uav.NavGPSMode
       transitions:
         complete: land

     land:
       mode: uav.LandingMode

.. note::
    Mode parameters are omitted here to keep the focus on how the behaviors connect.

The :doc:`Mode Manager <mode-manager>` executes the plan described by the
mission. It begins with ``start`` and activates the takeoff mode. When takeoff
reports ``"complete"``, the mission directs the Mode Manager to activate
``navigate``. The same process moves the mission from navigation to landing,
and the mission finishes once landing is complete.


For a complete mission file and implementation details, see
:doc:`Creating a Mission <../tutorials/create-mission>`.
