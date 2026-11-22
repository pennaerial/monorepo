:doc-status: evolving
:status-reason: Initialization has no ready result or automatic reconnect; hardware UART transport is unfinished.

The DDS Connection, Step by Step
================================

The Linux client connects to the Agent over UDP at ``127.0.0.1:7777`` and asks
it to create DDS entities in domain 0. A working UDP socket alone does not prove
that the session and reader/writer creation succeeded.

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
