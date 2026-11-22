Package Structure, Builds, and Change Workflow
==============================================

``ros_ws/src/common`` is a source grouping inside the single ``ros_ws`` workspace.
It is not a separate build workspace. Packages under common, IARC, and SAE can
depend on one another, so source-folder ownership and build dependency are
different views of the code.

How files become usable ROS packages
------------------------------------

.. mermaid::

   flowchart LR
       accTitle: ROS source to runtime
       accDescr: Package manifests declare dependencies, setup.py or CMake installs resources and entry points, colcon builds them, and the sourced overlay makes them discoverable at runtime.
       source["Source + package.xml"] --> build["colcon · setup.py / CMake"]
       build --> install["install · executables, libraries, resources"]
       install --> overlay["source install/setup.bash"]
       overlay --> runtime["ros2 run / launch / package lookup"]

A Python package's ``setup.py`` lists console scripts and data files. For example,
``uav`` installs ``uav_mission``, launch files, mission YAML, camera calibrations,
and its generated parameter module. A C++ package's CMake builds libraries or
executables, generates interfaces/parameters, and installs headers/plugin XML.
``package.xml`` declares package dependencies and build type.

Code can exist in source without being installed. A new mission needs an install
rule that includes it; a plugin needs a built library and exported description;
a new executable needs an entry point or installed target.

Ownership map
-------------

.. list-table:: Packages under ``src/common``
   :header-rows: 1
   :widths: 30 40 30

   * - Package
     - Owns
     - Detailed guide
   * - ``vehicle_common``
     - Mission schemas, registry, lifecycle, shared vehicle/launch helpers.
     - :doc:`mission-loading`, :doc:`mode-runtime`.
   * - ``uav``
     - Manager, PX4 adapter, shared modes and launches.
     - :doc:`uav-runtime`, :doc:`vehicle-api`, :doc:`flight-modes`, :doc:`launch`.
   * - ``sim`` / ``sim_interfaces``
     - World setup, entities, stage configuration, ROS simulation services.
     - :doc:`simulation`.
   * - ``pennair_gz`` / ``pennair_gz_msgs`` / ``sae_gz_cmake``
     - Simulator systems, custom protobufs, bridges, Gazebo CMake support.
     - :doc:`gazebo`.
   * - ``camera_ros`` / ``pennair_vision``
     - Camera input and vision plugins.
     - :doc:`vision` (WIP).
   * - ``pennair_planner``
     - Path-planning service and algorithm plugins.
     - :doc:`planning` (WIP).
   * - ``pennair_cli`` / ``tools``
     - Discovery commands, teleop, camera debugging, test ground station.
     - :doc:`tools`.
   * - ``px4_msgs``
     - PX4 ROS message definitions in a submodule.
     - :doc:`vehicle-api` for actual endpoints.

Some interface packages live under ``src/sae`` even when common code uses them.
Dependencies should be explicit in manifests; do not move interfaces between
folders without reviewing every consumer and generated type name.

Build the right scope
---------------------

Use :doc:`../getting-started` for the first full environment setup. Then, from
``ros_ws/`` with ROS sourced:

.. code-block:: bash

   colcon list
   colcon build --packages-up-to uav
   source install/setup.bash
   ros2 pkg executables uav
   ros2 pkg prefix --share uav

``--packages-up-to`` includes declared workspace dependencies. ``--packages-select``
assumes those are already available. Neither can fix undeclared launch-time
requirements automatically. ``--symlink-install`` helps source iteration, but
new files, schemas, entry points, and installed resources still need rebuilds.

Keep ``build/``, ``install/``, and ``log/`` separate from source. If launch sees an
old YAML, inspect the installed share path before deleting all build caches.
Source the intended overlay in every terminal; another workspace sourced later
can shadow it.

Parameters, interfaces, and plugins
-----------------------------------

* **Generated parameters:** edit the source schema and rebuild its owning
  package. Read-only parameters are set at node creation, not changed afterward.
* **ROS msg/srv interfaces:** rebuild the interface package and its consumers.
  Both endpoints need compatible definitions; a stale overlay can hide a mismatch.
* **PX4 messages:** align the ``px4_msgs`` version with the checked-out PX4
  firmware/DDS exports. Topic suffixes and fields can change between versions.
* **Plugins:** keep the library target, C++ export, XML name, base class, and
  package export consistent. Source a rebuilt overlay before testing discovery.

Normal tests versus live simulation
-----------------------------------

.. code-block:: bash

   colcon test --packages-select vehicle_common uav --return-code-on-test-failure
   colcon test-result --verbose

``vehicle_common`` tests exercise mission/schema/registry behavior with mocks.
The UAV package includes separate ``sim_test_*.py`` tests that launch a complete
PX4/Gazebo session. The basic test observes arming, airborne state, and landing;
the tailsitter test exercises its configured mission/transition behavior.
They need the full simulator environment and a MAVLink peer helper.

Normal test collection does not establish live-flight behavior. Follow
:doc:`../../ci/checks` for the separate ``ctsim`` command and final test-result
check. CI currently rebuilds club-owned ROS packages even in its "quick" mode;
local colcon incremental builds behave differently.

Trace a change through the system
---------------------------------

For a navigation change: locate the YAML mode ID → registered class/schema →
mode update → vehicle method → outgoing topic → feedback callback → completion
condition. Change the owning layer, test its boundary, then run the affected
simulation. For a perception change, identify its output consumer before adding
new messages. For launch changes, check installed resources and
``--show-args`` before starting the stack.

Useful source entry points: `package manifests and sources <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/common>`_,
`UAV install rules <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/uav/setup.py>`_,
`planner CMake <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_planner/CMakeLists.txt>`_.
