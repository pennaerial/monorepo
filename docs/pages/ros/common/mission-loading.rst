:doc-priority: key
:key-reason: Read before editing mission structure, mode registration, or parameter validation.

Mission Files, Validation, and Mode Discovery
=============================================

This page follows a mission from YAML to Python objects. It covers
``vehicle_common/mission_loader.py`` and ``mode_loader.py``; execution is covered
separately in :doc:`mode-runtime`. Use :doc:`../getting-started` for the
initial workspace setup.

How loading connects to execution
---------------------------------

.. mermaid::

   flowchart TB
       accTitle: Mission loading pipeline
       accDescr: Installed YAML is parsed and validated. Mode registry entries supply classes and parameter schemas before the manager initializes behavior instances.
       file["Installed mission YAML / explicit path"] --> yaml["YAML → RuntimeMission"]
       imports["Import mode modules"] --> decorators["register_mode → registry entries"]
       decorators --> validate["Resolve each mode ID and validate params"]
       yaml --> validate
       validate --> targets["Intersect compatible vehicle targets"]
       targets --> manager["Vehicle-specific manager"]
       manager --> instances["One initialized instance per local mode name"]

A minimal complete example
--------------------------

The following mission uses existing registered modes: take off, then land.
It is intended for the UAV simulation setup, not for launching arbitrary hardware.

.. code-block:: yaml

   modes:
     start:
       mode: uav.VerticalTakeoffMode
       params:
         takeoff_height: 3.0
         takeoff_method: PX4_AUTO
       transitions:
         complete: land
     land:
       mode: uav.LandingMode

``start`` and ``land`` are names **within this mission**. The string after
``mode:`` is a **registry ID** for reusable code. Two local entries can use the
same registered class with different parameters; they become separate instances.
A transition maps the active mode's outcome to another local name.

Locate the actual file
----------------------

``get_mission_path(name, package)`` asks the ament package index for the installed
package share directory, then appends ``missions/<name>.yaml``. Launch resolves
an installed mission first and can fall back to a supplied YAML filepath.
Editing a source YAML does not necessarily update its installed copy.

With ROS and the workspace sourced:

.. code-block:: bash

   ros2 pkg prefix --share uav
   ros2 run pennair_cli pennair mission uav
   ros2 run pennair_cli pennair mission uav basic

To try a custom file, save the example above as ``/tmp/common-takeoff-land.yaml``.
With the simulator prerequisites and QGroundControl prepared as in
:doc:`../getting-started`:

.. code-block:: bash

   ros2 launch uav uav_sitl.launch.py \
     mission:=/tmp/common-takeoff-land.yaml mission_package:=uav \
     airframe:=quadcopter auto_launch:=false

This validates and launches the stack; use the start service in
:doc:`uav-runtime` to begin the simulated mission.

What validation actually checks
-------------------------------

``RuntimeMission.load_from_path`` calls ``yaml.safe_load`` and Pydantic's
``model_validate``. A mission has a ``modes`` dictionary. Each ``RuntimeMode``
contains ``mode``, ``params`` (default empty), and ``transitions`` (default empty).
Both outer models forbid unknown fields, catching misspellings at those levels.

Each entry then:

1. Resolves its ID through ``ModeRegistry.get().get_registered_mode``.
2. Validates its parameter dictionary using the registered parameter class.
   For example, ``NavGPSParams`` requires ``coordinates`` and constrains the
   coordinate-system string to ``GPS`` or ``LOCAL``.
3. Stores the resolved registry entry and validated parameter object as private
   attributes, so execution uses the same resolved types.

The rules **inside params** belong to that mode's Pydantic class. ``ParamsBase``
does not itself forbid extra parameter fields, so do not assume every typo
inside ``params`` is rejected. Add explicit validation to the owning schema
when a field needs a range or a strict set of values.

The mission intersects every mode's declared ``targets``. The current UAV manager
requires the exact ``UAV`` type in that intersection. This is not automatic
subclass compatibility or a check of the actual airframe: VTOL-specific modes
currently also declare ``UAV``. Choose an appropriate vehicle yourself.

Validation does not prove there is a ``start`` entry, that all transitions exist,
or that every returned outcome has a mapping. An empty mission is also not a
useful input for the current intersection code. Inspect the graph before running.

How the registry gets populated
-------------------------------

``@register_mode`` records the mode ID, Python class, parameter class, targets,
and transition labels in a pending list when Python imports its module.
``ModeRegistry.get`` creates a singleton on first use and discovers modes under:

* ``uav.modes`` for shared flight behaviors;
* ``payload.modes`` for ground-payload behaviors;
* ``iarc_flight.modes`` for IARC behaviors.

Discovery recursively imports submodules, then puts pending entries into its
ID-to-description dictionary. Duplicate IDs raise an error. Import failures are
printed and discovery can continue, so one missing optional solver/library can
leave a mode absent from an otherwise usable registry. Read earlier import
messages when the final error says an ID was not found.

The registry does not scan every installed package. For a new mode, place it in
a discovered module or deliberately extend discovery. Restart the process after
editing mode registrations; importing a module is normally a one-time event in
that process. ``write_json`` is available for inspection, but the current runtime
discovers Python modules rather than relying on a generated registry JSON file.

Inspect registered entries with:

.. code-block:: bash

   ros2 run pennair_cli pennair mode ls

Then read :doc:`mode-runtime` to see how the resolved classes are instantiated.
For authoring, continue to :doc:`../../tutorials/create-first-mode` and
:doc:`../../tutorials/create-mission`.

Sources: `mission_loader.py <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/vehicle_common/vehicle_common/mission_loader.py>`_
and `mode_loader.py <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/vehicle_common/vehicle_common/mode_loader.py>`_.
