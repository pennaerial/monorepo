Project Organization
====================

Read :doc:`overview` for runtime flow. This page maps that flow onto files.
All paths below are relative to ``payload_controller/``.
A ``.hpp`` header declares the types and functions other files can use; a ``.cpp``
file contains their implementation. A **component** groups related code into a
buildable module. A **dependency** is another component or library that it needs.
**CMake** reads the build descriptions; **ESP-IDF** supplies the board-specific
build tools and runtime. See :doc:`index` for the simulation/hardware targets.

.. list-table:: Source map
   :header-rows: 1
   :widths: 30 40 30

   * - Location
     - Responsibility
     - Continue reading
   * - ``main/payload_controller.cpp``
     - Entry point and target-specific startup
     - :doc:`payload`
   * - ``components/payload/``
     - Payload orchestration and EKF state
     - :doc:`estimation/index`
   * - ``components/drivers/``
     - IMU, motor, encoder, and camera interfaces/backends
     - :doc:`sensors/index`
   * - ``components/DDS/``
     - XRCE session, topic registry, IDL, generated serializers
     - :doc:`dds_client/index`
   * - ``components/sitl/``
     - Gazebo transport initialization, runtime names, message conversion
     - :doc:`sensors/imu/sitl`
   * - ``components/util/``
     - PID, feedforward, PWM composition, mutex and logging helpers
     - :doc:`control/index`
   * - ``Makefile``, ``CMakeLists.txt``, ``cmake/``
     - Build entry points and shared CMake helpers
     - :doc:`build-system`
   * - ``scripts/sim.sh``
     - World startup, model spawn, executable launch, process cleanup
     - :doc:`getting-started`
   * - ``sdkconfig.*``, ``sdkconfig.defaults*``
     - Per-target resolved settings and initial defaults
     - :doc:`build-system`
   * - ``build/<target>/``
     - Generated build cache, objects, binaries, message C code
     - Rebuild from source; do not hand-edit

Dependency direction
--------------------

``main -> payload -> drivers / DDS / util`` is the main application dependency
chain. The payload component depends on ``DDS``, ``encoder``, ``imu``, ``motor``,
``camera``, ``util``, and IDF logging/FreeRTOS. Linux driver implementations use
``sitl`` and Gazebo libraries; hardware implementations use ESP peripheral drivers.
``DDS`` supplies generated message headers also used as local data structures.
Using a message structure locally does not mean every sample travels over DDS.

A **component** is a buildable module with a ``CMakeLists.txt``; it is not
necessarily a single class. ``payload`` contains both Payload and EKF.
``components/drivers`` is a grouping directory; its children are the components
found through ``EXTRA_COMPONENT_DIRS`` in the top-level CMake file.

How to trace an unfamiliar feature
----------------------------------

1. Find the public interface in a ``.hpp`` file: inputs, units, and ownership.
2. Follow its call sites in ``payload.cpp`` or the entry point.
3. Read its component CMake file to determine which backend is actually compiled.
4. Follow output consumers: cache, estimator, controller, DDS, or physical pins.
5. Use :doc:`reference/index` for exact signatures and :doc:`components` to change
   the dependency graph.

Source: `project directory <https://github.com/pennaerial/monorepo/tree/main/payload_controller>`_.

For a file-by-file reading index, use :doc:`code-map`. Shared threading helpers
are explained in :doc:`runtime-helpers`, simulation setup in :doc:`simulation`,
and analysis/test limitations in :doc:`development-tools`.
