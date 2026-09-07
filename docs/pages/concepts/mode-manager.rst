Mode Manager
============

.. rst-class:: lead

   The Mode Manager executes a mission by running one mode at a time and moving
   between modes based on their outcomes.

----

What Is the Mode Manager?
`````````````````````````

The Mode Manager coordinates mission execution for one vehicle. It keeps track
of the active :doc:`mode <modes>`, runs that mode, and responds when the mode
reports an outcome.

The :doc:`mission <missions>` describes the overall plan, and each mode
performs one vehicle behavior within that plan. The Mode Manager brings the two
together by carrying out the mission with the available modes.

.. note::
    The Mode Manager does not perform takeoff, navigation, landing, or other vehicle
    behaviors itself. It also does not decide the order of those behaviors. That
    logic remains separated between the modes and the mission.


Executing a Mission
```````````````````

Consider this simplified mission:

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

The Mode Manager executes it as follows:

#. Begin at ``start`` and activate ``uav.vtol.TakeoffMode``.
#. When takeoff reports ``"complete"``, follow its transition to ``navigate``.
#. Exit the takeoff mode and activate ``uav.NavGPSMode``.
#. When navigation reports ``"complete"``, follow its transition to ``land``.
#. Exit navigation mode and activate ``uav.LandingMode``.
#. When landing reports ``"terminate"``, end the mission safely.

Only one mode is active for the vehicle at a time. When the mission moves
forward, the current mode exits before the next mode enters. This gives every
mode a clear beginning and end.


Responding to Outcomes
``````````````````````

An outcome tells the Mode Manager what happened inside the active mode. Based on
that outcome, the Mode Manager either keeps the current mode active, switches to
the next mode described by the mission, or ends the mission safely.

How the Pieces Work Together
````````````````````````````

.. list-table::
   :header-rows: 1

   * - Component
     - Responsibility
   * - :doc:`Mode <modes>`
     - Perform one vehicle behavior and report its outcome.
   * - :doc:`Mission <missions>`
     - Define the overall plan and determine what follows each outcome.
   * - Mode Manager
     - Execute the plan by activating, running, and switching between modes.
