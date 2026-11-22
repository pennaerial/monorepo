:doc-status: evolving
:status-reason: Vision plugin configuration and downstream integration are evolving; the current plugins.xml has a malformed class entry that needs repair before reliable plugin loading.

Camera Inputs and Vision Plugins
================================

``pennair_vision`` provides a shared camera-processing host and plugin interface.
``camera_ros`` is the hardware camera driver; the UAV simulation can provide
bridged images instead. A vision result is not automatically used by a mission:
trace its subscriber before assuming it influences control.

Data flow and ownership
-----------------------

.. mermaid::

   flowchart TB
       accTitle: Vision input and lazy processing
       accDescr: Hardware or bridged image and CameraInfo feed a vision plugin. Subscriber demand decides whether to process each paired frame and produce results or throttled debug images.
       camera["camera_ros / Gazebo image bridge"] --> pair["Image + matching CameraInfo"]
       pair --> demand{"Result or due debug subscriber?"}
       demand -->|no| skip["Skip processing this frame"]
       demand -->|yes| algorithm["Plugin process(image, info, generate_debug)"]
       algorithm --> result["/cv/result"]
       algorithm --> debug["/cv/debug · rate limited"]

``vision.launch.py`` uses a multi-threaded ROS component container. It loads
``VisionManager`` and, when ``sim:=false``, a ``camera_ros`` CameraNode in the
same container with intra-process communication enabled. Simulation mode expects
an existing image/CameraInfo source and does not create the UAV camera bridge
itself.

Manager and plugin construction
-------------------------------

VisionManager reads its generated parameter listener, then its node factory
calls ``init_plugins`` after shared construction. Each requested class is loaded
through ``pluginlib::ClassLoader<VisionPlugin>`` and initialized with the host
node, camera topic, and optional debug rate. The plugin map owns their lifetimes.
Pluginlib creation failures are logged. An empty plugin list loads no algorithms.

The parameter schema supplies ``camera_topic`` (schema default
``/camera/image_raw``), ``plugins`` (empty), and ``debug_image_rate`` (5 Hz, read-only).
The launch file overrides the camera-topic default to ``camera/image_raw``.
Relative and absolute names therefore matter. Launch parses the plugin argument
as a Python list literal, not a comma-separated string.

.. warning::

   The current ``plugins.xml`` is not well-formed: the AprilTagPlugin class entry
   is missing its closing tag before VisionRepublisher. Also, its actual plugin
   identifiers are the fully qualified C++ class names. Reconcile the manifest
   before treating a sample launch as a working clean-checkout recipe. Use
   inspection commands below to distinguish manifest failure from camera failure.

Why processing is called "lazy"
-------------------------------

``LazyCameraPublisher<MessageT>`` implements the common callback in its header.
It creates absolute output topics ``/cv/result`` and ``/cv/debug`` and subscribes
to paired image/CameraInfo input through image_transport. On each paired frame:

1. Check whether anyone subscribes to the result topic.
2. Check debug subscribers and whether its steady-clock interval has elapsed.
3. Return without algorithm work if neither output is needed.
4. Call ``process`` with ``generate_debug`` true only when a debug frame is due.
5. Publish a nonempty requested debug image using the input header and returned
   encoding. Publish the typed result when result subscribers exist.

A result subscriber causes processing at the available input-frame rate; the
5 Hz setting limits debug generation, not result processing. With only debug
subscribers, processing runs only when debug is due. A rate of 0 becomes no
limit in the manager's optional-rate conversion. Debug attempts are throttled
even when the plugin returns no debug image.

The camera subscription still exists when there are no output subscribers; lazy
here means skipping expensive processing, not shutting down camera capture.
Multiple plugin instances share absolute output names; plugins with different
result types cannot be assumed to form one coherent typed output topic. Plan
remapping/output separation before combining them.

What the current plugins do
---------------------------

* **BasicVision:** increments an Int32 processing counter. If debug is requested,
  converts the image to grayscale and returns MONO8. It is a wiring example,
  not object detection.
* **VisionRepublisher:** wraps the input pixels for debug output while preserving
  their encoding. Use it to inspect camera output without an expensive detector.
* **AprilTagPlugin:** converts BGR to grayscale, detects tag36h11 markers, and
  records IDs, image corners/centre, decision metrics, and homography. It uses
  CameraInfo intrinsics and a currently hardcoded **0.165 m** tag size to estimate
  camera-relative pose. It converts the rotation matrix to a normalized ROS
  quaternion and marks whether the pose result exists. Debug adds outlines,
  IDs, and projected axes only when requested.

A tag pose is relative to the camera, not the UAV/world frame. Camera calibration,
physical tag size, and the camera-to-body transform all affect its interpretation.
Do not feed the result directly into navigation without establishing those
conventions and checking pose validity.

Inspect a configuration
-----------------------

With dependencies built and the workspace sourced:

.. code-block:: bash

   ros2 launch pennair_vision vision.launch.py --show-args
   ros2 interface show pennair_vision/msg/AprilTagArray
   ros2 topic list -t
   ros2 topic info /uav_0/camera --verbose
   ros2 topic info /uav_0/camera_info --verbose

After the manifest is repaired, a simulation configuration can select
``camera_topic:=/uav_0/camera``, ``sim:=true``, and
``plugins:="['pennair_vision::AprilTagPlugin']"``. It needs both image and
CameraInfo from a camera-equipped model. The launch also accepts camera format,
width, height, orientation for hardware; ``auto`` leaves detection to camera_ros.
Optional ``record_topics`` uses a list literal and records MCAP under ``~/bags/``.

For a new plugin, implement VisionPlugin or derive from LazyCameraPublisher,
choose the result message, implement ``process``, add the C++ export/XML entry,
and add sources/dependencies to CMake. Keep debug generation optional, report
units/frames, and test no-subscriber, debug-only, and result-subscriber behavior.

Sources: `vision launch <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_vision/launch/vision.launch.py>`_,
`lazy callback <https://github.com/pennaerial/monorepo/blob/main/ros_ws/src/common/pennair_vision/include/pennair_vision/lazy_camera_publisher.hpp>`_,
`plugin implementations <https://github.com/pennaerial/monorepo/tree/main/ros_ws/src/common/pennair_vision/src/plugins>`_.
