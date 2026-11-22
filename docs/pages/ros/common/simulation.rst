Simulation Worlds, Entities, and Time
=====================================

``common/sim`` assembles the simulated environment. It does not execute the UAV
mission: :doc:`launch` starts PX4 and the manager separately. World generation,
vehicle control, and simulation time meet here but have different owners.

Startup sequence
----------------

.. mermaid::

   flowchart TB
       accTitle: Simulator launch and world generation
       accDescr: Gazebo and a clock bridge start first. A matching installed stage adds a world node and spawn bridge. A Trigger request asks that node to enqueue entity creation.
       launch["sim2.launch.py"] --> gz["gz sim · world SDF"]
       launch --> clock["Gazebo clock → ROS /clock"]
       launch --> stage{"Installed stage YAML exists?"}
       stage -->|no| base["Base world only"]
       stage -->|yes| node["Configured world node"]
       node --> trigger["generate_world Trigger"]
       trigger --> entities["EntityFactory requests"]
       entities --> bridge["ROS SpawnEntity ↔ Gazebo create"]
       bridge --> gz

``sim2.launch.py`` builds the world path from
``PENNAIR_GZ_MODELS_PATH/worlds/<world>.sdf``. It supplies model-resource paths,
``gz-models/server.config``, and PX4's built Gazebo plugin directory. It then
starts ``gz sim --render-engine ogre -r`` plus the ``/clock`` bridge.

With an installed ``simulations/<world>/<stage>.yaml``, it adds the configured
world-node executable, a ``/world/<world>/create`` SpawnEntity bridge, and a
``ros2 service call`` process that triggers generation. Without that file it
logs a warning and leaves Gazebo/clock running without the dynamic world node.
A missing stage and a missing base-world SDF are different failures.

World and stage configuration
-----------------------------

``SimulationParams.load_from_stage`` reads the installed ``sim`` share directory.
``WorldParams`` carries a node executable, world name, entity/controllable lists,
optional physics, and a free-form world-specific config dictionary. Before
validation it injects the world name into each entity entry.

For example, SAE's base stage separates static entities from controllables,
but ``SAEWorldNode.generate_world`` currently spawns both lists in the same way.
Listing a model as controllable does not launch its ROS controller. That requires
a separate launch action.

``Entity`` resolves an explicit SDF/URDF filepath first. Otherwise it looks up
``gz-models/models/<model>/model.urdf``, falling back to ``model.sdf``. It validates
file existence and extension, then builds an EntityFactory request with the
name, pose, world reference, and inline SDF if present. Roll/pitch/yaw are
converted to a quaternion when creating the pose.

Adding an entity therefore requires a valid model resource and a stage entry;
adding its controller or mission is another change. ``sim/setup.py`` installs
stage YAML and other resources: rebuild and source before expecting edits in
an installed lookup.

Queued creation versus completed creation
-----------------------------------------

``WorldNode`` exposes ``/<node_name>/generate_world``. Its callback waits up to
10 seconds for the spawn bridge/service, then calls the concrete generator.
``spawn_entity`` uses ``call_async`` and returns true when a request is queued;
a done callback logs the eventual Gazebo response.

A successful generation Trigger can therefore mean requests were submitted,
not that all models already exist. Check per-entity completion logs or inspect
Gazebo before starting consumers that require those models. Rerunning generation
also needs a plan for existing names; it does not automatically reset the entire
world.

Inspect and launch the simulator alone
--------------------------------------

With the environment from :doc:`../getting-started` sourced:

.. code-block:: bash

   ros2 launch sim sim2.launch.py --show-args
   ros2 launch sim sim2.launch.py world:=sae stage:=base headless:=false

This starts an environment, not a UAV mission. The model files named by that
stage must exist. From another prepared terminal:

.. code-block:: bash

   gz topic -l
   ros2 topic list -t
   ros2 service list -t
   ros2 topic echo /clock --once

Use ``gz service -l`` to inspect the Gazebo side separately. For a custom world
node, the generation service follows the node name, not necessarily the world
name. Verify it with the service list before calling it.

Headless mode and graphics
--------------------------

Headless adds ``-s --headless-rendering`` and supplies software rendering and an
offscreen Qt platform. GUI mode clears inherited OpenCV Qt plugin/font paths
and selects ``xcb``. These choices help avoid GUI environment conflicts; camera
sensors can still need rendering even with no visible window.

Gazebo discovery defaults ``GZ_IP`` to ``127.0.0.1`` unless explicitly supplied,
so VPN interfaces do not become the local transport interface by accident. A
multi-machine simulation needs a deliberate network configuration rather than
assuming that local default reaches other hosts.

Which clock is being used?
--------------------------

The bridge publishes ROS ``/clock``. A node uses it only if configured for ROS
simulation time and if its code reads that clock. The shared mode manager uses
Python wall-clock ``time()`` for ``dt``; setpoint timestamps use the node clock;
VehicleCommand uses a fresh Clock; vision debug throttling uses steady wall time.
Pausing Gazebo does not automatically pause all of those operations.

When testing a timeout or dwell, record the time source, whether simulation is
paused, and whether telemetry is still updating. A nominal 10 Hz timer does not
mean every update is exactly 0.1 seconds apart under load.

Scoring and current limits
--------------------------

``ScoringNode`` offers score/status publishers and abstract update behavior.
``ScoringParams`` remains minimal, and the current ``sim2.launch.py`` does not
instantiate scoring from the stage's scoring field. Having a scoring class or
configuration is not evidence that a scoring node is running.

``scoring/namespacing.py`` includes helpers for another controllable mapping
shape, while current WorldParams uses lists. It also constructs an unversioned
local-position topic, whereas the current UAV adapter uses a ``_v1`` topic.
Check actual callers/schema before connecting those helpers to the launch path.
``WorldNode.set_physics`` is likewise not wired into the active dynamic-spawn
path; an optional friction value does not by itself prove runtime physics was
changed.

World-specific generation and scoring remain more volatile than the shared
mission lifecycle. Read their concrete subclasses before extending a stage.
For custom attachment systems and bridges, continue to :doc:`gazebo`.

Sources: `sim2 launch <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/sim/launch/sim2.launch.py>`_,
`simulation parameters <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/sim/sim/simulation_params.py>`_,
`WorldNode <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/sim/sim/world_gen/world_node.py>`_,
`Entity <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/sim/sim/entity.py>`_.
