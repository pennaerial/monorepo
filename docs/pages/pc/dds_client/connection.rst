:doc-status: evolving
:status-reason: Initialization has no ready result or automatic reconnect; hardware UART transport is unfinished.

The DDS Connection, Step by Step
================================

A **topic** is a named channel carrying one message type. A writer sends samples;
a reader receives them. DDS lets processes find matching writers/readers without
the application addressing each receiving process individually.

The small C++ controller uses **Micro XRCE-DDS**, a client protocol for devices
that cannot afford a full DDS implementation. An **Agent** runs on a computer and
creates DDS objects for the client. **UDP (User Datagram Protocol)** is the network transport carrying XRCE
packets between this Linux client and the Agent. These are different layers:
a working UDP **socket** (the program's endpoint for sending/receiving network packets) alone does not establish DDS readers, writers, or matching types.

What the objects mean
---------------------

.. list-table:: Objects created during startup
   :header-rows: 1
   :widths: 25 75

   * - Object
     - Purpose in this project
   * - Session
     - The XRCE connection identified by the client key; carries requests and responses.
   * - Domain participant
     - The Agent-side entry into DDS domain 0, the group of DDS participants that can communicate.
   * - Topic
     - A channel name plus a type name, such as ``rt/imu`` and its Imu DDS type.
   * - Publisher / subscriber
     - Containers that own writers / readers; they are not sensor samples themselves.
   * - DataWriter / DataReader
     - Endpoints that send / receive samples on a topic.
   * - Stream
     - An XRCE queue used to move requests or data between client and Agent.

Read ``DDSClient::init()`` in this order
----------------------------------------

1. Open a transport. Linux calls ``uxr_init_udp_transport`` with
   ``127.0.0.1:7777``. Hardware installs custom UART callbacks instead, but those
   callbacks do not yet transfer bytes.
2. Bind that transport to ``session_`` using the fixed client key ``0xABCDABCD``.
   Register ``on_topic_callback`` and pass ``this`` so the C callback can find the C++ object.
3. Call ``uxr_create_session``. This is the handshake with the Agent.
   The transport-opening success log does not prove this handshake succeeded.
4. Create reliable input/output streams using the class's fixed byte buffers.
   Reliable means XRCE tracks delivery between this client and Agent.
5. Queue creation of the participant, publisher, and subscriber. ``UXR_REPLACE``
   requests replacement of objects using the same XRCE identity.
6. ``generate_topics`` queues a Topic object for each registry entry.
   Writer and reader entries can use the same DDS channel name but have separate
   XRCE object keys in this code.
7. ``generate_writers`` creates endpoints only for WRITER entries.
   ``generate_readers`` creates endpoints only for READER entries, then requests
   their data on the input stream with no sample-count limit.
8. ``uxr_run_session_until_all_status`` sends the queued creation requests and
   waits up to 1000 ms in this call for their status results. If it fails,
   ``status_name`` turns each numeric result into a readable log label.

Queueing a create request does not mean the Agent has created that object yet.
The final session-running step makes progress on the queue and receives responses.
With four registry entries, there are 3 shared objects + 4 topics + 2 writers +
2 readers = **11 creation statuses**. Two request-data operations are also queued;
they do not receive CREATE-status entries. The header comment referring to 13
requests includes that additional startup traffic.

.. mermaid::

   sequenceDiagram
       accTitle: DDS startup and first outgoing sample
       participant App as Application
       participant Client as DDSClient
       participant Agent as XRCE Agent
       participant ROS as ROS reader
       App->>Client: init()
       Client->>Agent: Open XRCE session
       Agent-->>Client: Session response
       Client->>Agent: Create DDS objects and request reader data
       Agent-->>Client: Creation statuses
       App->>Client: publish(message)
       Note over Client: Serialize into output queue
       App->>Client: update()
       Client->>Agent: Run session and seek delivery confirmation
       Agent->>ROS: DDS sample if type and QoS match

Buffers and timing
------------------

``STREAM_HISTORY`` is 16 blocks, a power of two. Each input/output buffer has
``TRANSPORT_MTU * STREAM_HISTORY`` bytes; MTU is the maximum transport packet size
selected by the built library. A bigger history reserves more memory to keep
unconfirmed traffic. It is not the same as DDS topic history, which is set to one
sample here. The wrapper limits individual outgoing messages to one block, even
though the underlying library can support other delivery patterns.

``update()`` tries to confirm pending output with a 1000 ms wait, then runs the
session for 10 ms. Therefore DDS work can add appreciable delay to the application
loop. A successful confirmation concerns client-to-Agent delivery, not proof
that every ROS subscriber consumed the sample.

Failures you must distinguish
-----------------------------

``init()`` returns ``void`` and logs failures before returning early. There is no
ready flag checked by ``publish()`` or ``update()``, and no application-level
reconnect loop. Start the Agent first and read the session/entity logs. Restart
the application after correcting a failed startup rather than assuming it has
recovered automatically. Using several clients also requires distinct session
keys; changing only their model names does not change this fixed key.

Next: :doc:`topics`. Source:
`dds_client.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/dds_client.cpp>`_.
Further reading: `eProsima's stream and delivery guide
<https://micro-xrce-dds.docs.eprosima.com/en/stable/client.html>`_.
