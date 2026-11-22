Sending and Receiving Samples
=============================

Read :doc:`topics` for message types and encoding. This page follows the exact
call path for outgoing and incoming samples, including what a success result means.

An outgoing IMU sample
----------------------

``Payload::publish_sensor_debug`` reads a new cached IMU value with its own
freshness cursor. It calls:

.. code-block:: cpp

   DDSClient::instance().publish<TopicId::IMU_WRITER>(latest_imu);

The template parameter selects the registry entry and required message type.
``publish()`` is defined in ``dds_client.hpp`` so the compiler can instantiate it
for each selected ID. It performs these steps:

1. Reject an entry whose direction is not WRITER or whose size/serialize hooks
   are missing.
2. Compute the message's serialized byte size. Reject it if that size plus the
   32-byte estimated framing allowance exceeds one reliable-stream block.
3. Ask ``uxr_prepare_output_stream`` for queue space associated with the DataWriter.
   A full stream returns an invalid request ID; the wrapper logs and drops this sample.
4. Serialize the fields into the returned CDR buffer. Failure returns ``false``.
5. Set ``delivery_pending_`` and return ``true``.

**True means queued and serialized**, not received by ROS. The next ``update()``
tries ``confirm_delivery()`` and runs the session. On timeout, the pending flag
stays true so a later update tries again. The code does not store a separate
application retry queue for samples dropped before they fit into the stream.
It also does not currently check a connection-ready flag.

An incoming sample
------------------

The Agent forwards requested reader data through the XRCE input stream.
``on_topic_callback`` is a static C-compatible function. It casts its ``args``
back to the ``DDSClient`` pointer registered during startup and calls
``handle_topic``. That function:

1. Checks the incoming object is a DataReader and its numeric key is within the
   registry. It also checks the entry's direction.
2. Finds the entry's generated deserialize function.
3. Creates a stack buffer large enough for the largest configured message and
   aligned for ordinary message types with ``alignas(std::max_align_t)``.
4. Deserializes into that buffer, then finds the application's registered callback.
5. Calls it with the Topic, message pointer, received byte length, and user-supplied context.

``max_topic_message_size`` calculates the required storage across the registry.
The callback must interpret the pointer as the entry's actual C message type.
The ``length`` argument is received serialized length, not ``sizeof`` the struct.
**Copy the message during the callback** if it will be used later: the stack
buffer is gone once ``handle_topic`` returns.

Register a reader callback
--------------------------

This example copies the received message into storage owned by the application.
Run initialization first, then register the callback before normal session servicing:

.. code-block:: cpp

   struct ImuInbox {
     sensor_msgs_msg_Imu latest{};
     bool has_sample = false;
   };

   static void receive_imu(const Topic&, const void* msg, uint16_t, void* args)
   {
     auto& inbox = *static_cast<ImuInbox*>(args);
     inbox.latest = *static_cast<const sensor_msgs_msg_Imu*>(msg);
     inbox.has_sample = true;
   }

   // Keep this storage alive while callbacks can run.
   static ImuInbox inbox;
   DDSClient::instance().set_reader_callback(
       TopicId::IMU_READER, receive_imu, &inbox);

Check the registration result in real application code. This example assumes
one owning thread; add synchronization if another thread reads the inbox.
The current application does not register these callbacks itself, so declaring
reader entries alone does not connect received messages to the EKF or motors.

Reader callbacks can run while session functions execute, including startup or
delivery waits. Keep them short and avoid recursive publish/session calls. The
built client has multithread support disabled; the sensor-cache mutexes do not
make DDSClient thread-safe. Prefer one owner for DDS calls and separate queues
or caches for other tasks.

Next: :doc:`development`. Lookup: :doc:`../reference/communications`.
