Topics, Types, and Message Bytes
================================

``topics.h`` is the registry that ties channel names to message types and the
functions that encode/decode them. Read :doc:`connection` for their startup order.

Three names that look similar
-----------------------------

For the IMU writer, the registry uses:

* ``TopicId::IMU_WRITER``: a C++ identifier used in this application.
* ``rt/imu``: the DDS channel name. ROS tools normally show this as ``/imu``.
* ``sensor_msgs::msg::dds_::Imu_``: the DDS wire-type name used for matching.

The generated C struct is named ``sensor_msgs_msg_Imu``. Its C identifier is not
the DDS type string. Matching a topic name alone is insufficient: peers also need
compatible message layout and QoS.

What each registry piece does
-----------------------------

``Topic`` stores the ID, display name, DDS channel/type names, QoS, direction,
message-buffer size, and three function pointers: serialized size, serialize,
and deserialize. These function pointers let the common runtime call the correct
operation without an IMU-specific branch everywhere.

``SpecificTopic<MessageT>`` carries the C++ message type alongside those fields.
``TopicTypeAdapter`` bridges the typed generated C functions to the common
``void*`` function signatures. ``make_topic`` assembles the fields and adapters.
You normally add a registry entry with ``make_topic``, not hand-written casts.

``TYPED_TOPICS`` holds entries with different message types in a tuple.
``make_topic_array`` turns it into the uniform ``TOPICS`` array
used by runtime loops. ``TopicMsg<Id>`` looks back into the typed tuple so that
``publish<Id>`` expects the right C struct for that ID.

The ``TopicId`` numeric values must equal their positions in that tuple. The
static assertion checks the order. ``object_key``, ``topic_id``, ``datawriter_id``,
and ``datareader_id`` build XRCE identities from those values plus an object kind.
A topic and writer can reuse a numeric key because their kinds differ.
``get_topic`` asserts on an out-of-range ID in assertion-enabled builds, so do
not treat arbitrary external numbers as validated enum values.

What serialization does
-----------------------

A C++ object in memory may contain padding, arrays, or platform-specific layout.
A receiver cannot safely interpret an arbitrary memory dump as the same struct.
Generated functions and Micro-CDR encode the fields using CDR. Size calculation includes required alignment
and is separate from ``sizeof(MessageT)``.

``generate.py`` runs ``microxrceddsgen`` on the IDL files during CMake configuration.
The generated C headers, source files, and serialization functions live in the target build tree.
The generator preserves the IDL's relative package folders and fails the configure
step if a subprocess fails. Its repository/tool paths come from ``dev_env.sh``.

The six IDL files
-----------------

* ``builtin_interfaces/msg/Time.idl``: seconds and nanoseconds.
* ``std_msgs/msg/Header.idl``: timestamp and frame identifier.
* ``geometry_msgs/msg/Vector3.idl``: X/Y/Z values.
* ``geometry_msgs/msg/Quaternion.idl``: orientation coefficients.
* ``sensor_msgs/msg/Imu.idl``: header, orientation, gyro, acceleration, and covariance arrays.
* ``payload_interfaces/msg/MotorState.idl``: encoder counts plus many speed/control debug fields.

The current Payload publishing code fills the IMU message and left/right encoder
counts in MotorState. Most other MotorState fields stay at their zero-initialized
values; zero in those fields does not mean the controller measured zero speed or
has zero error. Read :doc:`../payload` before using them to tune a motor.

The configured QoS is volatile, best effort, keep last 1. These settings describe
the Agent's DDS endpoints; reliable XRCE streams govern the separate client-to-Agent link.

Next: :doc:`sending-receiving`. Source:
`topics.h <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/topics.h>`_.
