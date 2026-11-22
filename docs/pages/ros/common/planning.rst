:doc-status: evolving
:status-reason: The planner has implemented algorithms, but costmap production, mission integration, and plugin configuration are evolving.

Path Planning: Service, Plugins, and A*
=======================================

``pennair_planner`` computes a path between poses using a selected plugin.
It does not command motors or automatically connect itself to a mission.
A caller must provide a map, request a path, check the result, and arrange for
another component to follow it.

How a request becomes a path
----------------------------

.. mermaid::

   flowchart LR
       accTitle: Path-planning request and map flow
       accDescr: The service caches an occupancy grid, passes a snapshot to the selected planner with start and goal poses, then returns a Path and publishes successful plans for inspection.
       map["costmap · OccupancyGrid"] --> cache["Latest map snapshot"]
       request["plan_path · start and goal"] --> service["PlannerService"]
       cache --> service
       service --> plugin["AStarPlanner / StraightLinePlanner"]
       plugin --> path["Response Path + planned_path topic"]

``PlannerService`` is a composable rclcpp node with a standalone executable
``planner_service_node``. It subscribes to relative ``costmap`` using reliable,
transient-local depth 1 and publishes relative ``planned_path`` with
transient-local depth 10. A mutex protects the cached map pointer.
``planner_plugin`` defaults to the XML alias ``AStarPlanner``.

The ``plan_path`` service takes two unstamped Pose values and returns only a
``nav_msgs/Path``. There is no success flag or request-frame field. The caller
must express both poses in the costmap's frame.

The callback copies the latest map pointer under the mutex, rejects missing
maps or empty map frame IDs, sets the path frame from that map, and calls
``set_costmap`` then ``plan``. On success it returns/publishes the path. A plugin
failure clears/publishes an empty path. The earlier missing-map branch returns
an empty response without publishing a clearing message, so an old latched
visualization can remain visible. Check the service response, not just RViz.
Planning runs synchronously in the service callback and can block its executor.

A* in plain language
--------------------

The map divides the ground into cells. A* searches possible steps from start to
goal, preferring low total estimated cost. It keeps ``g`` (cost paid so far), a
heuristic ``h`` (estimated remaining distance), and orders work by ``f = g + h``.
The code implements this with a priority queue, best-cost array, and parent array.

The actual implementation:

1. Validate positive map dimensions/resolution and the expected data length.
   Reject rotated map origins; this implementation assumes an axis-aligned grid.
2. Convert world XY into cell indices by subtracting origin, dividing by
   resolution, and flooring. Reject out-of-bounds start/goal.
3. Treat occupancy values at or above 65 as obstacles. Inflate them by a default
   0.35 m radius so a path has clearance rather than passing a vehicle centre
   directly against an obstacle.
4. Search eight neighboring cells. Cardinal steps cost one cell width; diagonal
   steps cost √2 widths. Reject diagonal moves through blocked corners.
5. Multiply step cost by an occupancy penalty. Unknown cells are traversable
   with default multiplier 3; known cells below threshold get a scaled penalty.
6. Use an octile-distance heuristic, matching eight-direction movement. Replace
   a cell's recorded route when a cheaper one is found; ignore stale queue entries.
7. Follow parent indices from goal back to start, reverse them, and convert cell
   centres into poses. Restore exact endpoint positions, point intermediate
   headings toward the next waypoint, and use the requested final orientation.

For 0.1 m cells, one cardinal step covers 0.1 m and a diagonal about 0.141 m.
A 0.35 m inflation radius considers offsets up to four cells away, retaining
only offsets within the actual radius. Unknown does not mean blocked here;
changing that policy materially changes where paths can go.

Default configuration values are C++ Config fields (threshold 65, unknown cost 3,
occupancy scale 2, inflation 0.35 m). The service currently declares only the
plugin-choice parameter; editing an arbitrary ROS parameter does not tune all
those fields automatically. This is a 2D ground path, not a 3D flight planner or
a time-parameterized trajectory.

Straight-line plugin
--------------------

``StraightLinePlanner`` samples position linearly and interpolates relative
rotation using Eigen quaternions. Default 10 steps produce 11 poses including
both endpoints. A near-zero quaternion falls back to identity; other inputs
are normalized.

It does not check obstacles. The service still requires a valid map/frame before
calling it because that check is in the common service callback. Use it to
understand the plugin contract, not to claim collision-free navigation.

Build and inspect
-----------------

From ``ros_ws/`` with ROS and dependencies available:

.. code-block:: bash

   colcon build --packages-up-to pennair_planner
   source install/setup.bash
   ros2 interface show pennair_planner/srv/PlanPath
   ros2 run pennair_planner planner_service_node

In a second sourced terminal:

.. code-block:: bash

   ros2 param get /planner_service planner_plugin
   ros2 topic info /costmap --verbose
   ros2 topic echo /costmap --qos-durability transient_local --once

After a costmap producer supplies a valid map, choose start/goal values inside
its free area and call ``/plan_path``. Inspect the request schema with
``ros2 interface show`` first; a response with zero poses signals failure.
No map producer is started by the planner command itself.

To select the simple plugin, stop the first node and restart with:

.. code-block:: bash

   ros2 run pennair_planner planner_service_node \
     --ros-args -p planner_plugin:=StraightLinePlanner

For another algorithm, derive PathPlanner, export the plugin in C++ and XML,
add its sources to CMake, honor the caller's path header, and return false for
failure. Test blocked goals, unknown cells, map bounds, frame agreement, and
empty results before integrating with a controller.

Sources: `service <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_planner/src/planner_service.cpp>`_,
`A* <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_planner/src/planners/astar_planner.cpp>`_,
`planner interface <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_planner/include/pennair_planner/path_planner.hpp>`_.
