Gazebo Plugins, Messages, and ROS Bridges
=========================================

``pennair_gz`` contains code that runs inside Gazebo plus ROS nodes that bridge
custom requests. ``pennair_gz_msgs`` generates the custom protobuf messages.
These are different from ROS interface packages such as ``sim_interfaces``.
Start with :doc:`simulation` to understand the surrounding launch.

Two processes, two interfaces
-----------------------------

.. mermaid::

   flowchart LR
       accTitle: Custom attachment request path
       accDescr: A ROS request can be converted by the bridge to a Gazebo protobuf request. The simulator plugin queues changes and applies them during PreUpdate before publishing actual attachment state.
       caller["ROS caller"] --> bridge["AttachDetachBridge"]
       bridge -->|"Gazebo request"| service["DynamicDetachableJoint service"]
       service --> queue["Pending attach / detach"]
       queue --> update["Gazebo PreUpdate · joint change"]
       update --> state["Attachment state topic"]

An SDF plugin entry loads a simulator system into a model. ROS pluginlib loads
a separate bridge class into the bridge-loader process. Registering one does
not register or launch the other.

DynamicDetachableJoint
----------------------

``Configure`` resolves the parent model/link, reads the attachment-distance
threshold, advertises the request service, and creates a state publisher.
The default service name is
``/model/<parent_model>/dynamic_detachable_joint/attach_detach``; SDF can override
it. ``parent_link`` must identify the intended link.

The request contains child model name, child link name, and ``attach`` or
``detach``. The service callback validates names/command and records a pending
request under a mutex. Attachment physics is changed later in ``PreUpdate``:

1. Find the requested child model/link.
2. Calculate the parent-to-child link distance and reject an attach beyond the
   configured threshold.
3. Create a detachable-joint entity with the fixed joint component and remember
   the attached pair.
4. On a matching detach request, request removal of that joint entity, publish
   the detached state, and clear the remembered pair.

Already attached/detached cases and mismatched detach names are handled in the
service callback. A successful **accepted request** still precedes the distance
check and entity lookup in PreUpdate. Observe the state topic and simulator
logs to establish actual attachment; service transport success alone is weaker.

Build and discover the Gazebo side
----------------------------------

From ``ros_ws/`` with dependencies installed and ROS sourced:

.. code-block:: bash

   colcon build --packages-up-to pennair_gz
   source install/setup.bash
   gz service -l
   gz topic -l

The model must load the plugin and Gazebo must find its built shared library.
Inspect the SDF and package environment/plugin-path hook when no service appears.
``gz service --help`` explains how to inspect/call the actual protobuf service.
Use the names/types advertised by your build: the older README's protobuf
namespace can differ from current source's ``pennair_gz::msgs`` types.

ROS bridge implementation
-------------------------

``bridge_loader`` takes a short plugin name, prefixes
``pennair_gz::bridge::``, loads it through pluginlib, and spins the resulting node.
The package-level CMake exports ``bridge/plugins.xml`` for discovery.
``AttachDetachBridge`` reads generated parameters and requires nonempty
``ros_service_name`` and ``gz_service_name``.

Its callback converts the ROS request into AttachDetachRequest protobuf fields,
issues a blocking Gazebo request with a 5000 ms timeout, then copies the protobuf
success/message back into the ROS response. Timeout/transport failure returns
false with a corresponding message. This bridge reports the simulator service's
acceptance response, not delayed physical joint completion.

The ROS interface is ``sim_interfaces/srv/AttachDetach.srv``: child model,
child link, and command strings in the request; success and message in the
response. Rebuild this interface before its bridge consumers after a contract
change. A stale generated header can otherwise hide a schema mismatch.

Inspect the installed bridge executable and interface with:

.. code-block:: bash

   ros2 pkg executables pennair_gz
   ros2 interface show sim_interfaces/srv/AttachDetach

Adding another custom bridge
----------------------------

Define the simulator message/service and its ROS equivalent, write conversions,
and decide whether success means queued, accepted, or completed work. Add its
source to the bridge library, export its class in XML and C++, and register any
new generated parameter schema/dependencies. Launch it with explicit ROS/Gazebo
names and test both request failure and eventual simulator behavior.

Keep standard image/clock/spawn bridges in ``ros_gz_bridge`` where supported;
custom code is needed for custom contracts, not for every Gazebo topic.

Sources: `attachment system <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_gz/plugins/dynamic_detachable_joint/DynamicDetachableJoint.cpp>`_,
`ROS bridge <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_gz/bridge/attach_detach_bridge.cpp>`_,
`custom messages <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/common/pennair_gz_msgs>`_.
Further reading: `Gazebo system-plugin lifecycle <https://gazebosim.org/api/sim/8/createsystemplugins.html>`_.
