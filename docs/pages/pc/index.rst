:human-verified: true
:verified-by: User (review confirmed in chat)
:verified-on: 2026-11-21

Payload Controller ESP
======================

Learn the C++ project in ``payload_controller/`` from first build to sensor fusion
and motor control. The ``linux`` target runs on your computer for simulation;
``esp32s3`` builds firmware for the physical board. Their current feature coverage
is described in :doc:`overview`.

Use :doc:`glossary` for vocabulary and tool definitions. The guides focus on how
the code works; algorithm tutorials explain concepts where they are needed.

Start with :doc:`overview` to see how the pieces interact, then :doc:`getting-started`
to run them. Follow the pages in order for an introduction, or jump to
:doc:`reference/index` for function signatures. This is a different project from
the Python ROS package at ``ros_ws/src/sae/payload``.

Key reading for your task
-------------------------

Blue **Key** markers identify core architecture and behavior pages. Start with
:doc:`overview` and :doc:`project-organization`, then choose a path:

* **Drivers:** :doc:`sensors/index` → the device/backend page → :doc:`payload`.
* **Communication:** :doc:`dds_client/index` → connection/topics → sending/receiving.
* **Estimation:** :doc:`estimation/error-state-ekf` → implementation/integration.
* **Motor control:** :doc:`control/index` → feedforward/PID/PWMController.
* **Component/build changes:** :doc:`components` → :doc:`build-system` as needed.

Key describes reading priority; WIP and human verification remain independent.
Setup, vocabulary, and API lookup pages remain available without Key markers.

.. toctree::
   :maxdepth: 3

   overview
   glossary
   getting-started
   project-organization
   components
   build-system
   dds_client/index
   sensors/index
   payload
   runtime-helpers
   simulation
   estimation/index
   control/index
   development-tools
   code-map
   reference/index
