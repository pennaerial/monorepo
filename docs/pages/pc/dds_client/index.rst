:doc-priority: key
:key-reason: Start here for DDS work: separate local sensor data from the Agent and message-publication path.
:doc-status: evolving
:status-reason: The simulation uses a working UDP network connection. The hardware UART serial connection is unfinished; the client identity and address are fixed.

DDS Client
==========

The DDS component manages the XRCE session, message encoding, and reader/writer
endpoints used by Payload. Gazebo driver traffic is a separate path; the EKF
reads local sensor caches without depending on DDS delivery.

.. mermaid::

   flowchart TB
       accTitle: Gazebo and DDS are separate transport paths
       accDescr: Gazebo messages enter driver caches and Payload. Payload debug data then uses serialization, XRCE UDP, and an Agent to reach ROS DDS.
       gz["Gazebo sensors"] -->|"Gazebo Transport"| driver["Backend callbacks"]
       driver --> cache[("Local sensor caches")]
       cache --> payload["Payload"]
       payload -->|"local measurements"| ekf["EKF and speed control"]
       payload -->|"debug C messages"| cdr["Generated CDR serializers"]
       cdr -->|"XRCE · UDP 7777"| agent["Micro XRCE-DDS Agent"]
       agent -->|"DDS domain 0"| ros["ROS / DDS participants"]
       classDef sensor fill:#e0f2fe,stroke:#0284c7,color:#0c4a6e
       classDef compute fill:#ede9fe,stroke:#7c3aed,color:#3b0764
       classDef comm fill:#dcfce7,stroke:#16a34a,color:#14532d
       class gz,driver,cache sensor
       class payload,ekf compute
       class cdr,agent,ros comm

Start and inspect
-----------------

Start the Agent before the Linux application:

.. code-block:: bash

   MicroXRCEAgent udp4 -p 7777

Then follow the WIP manual simulation workflow in :doc:`../getting-started`. The current
client uses ``127.0.0.1:7777`` and DDS domain 0. The session key is the fixed
``0xABCDABCD``; multiple simultaneous clients need distinct keys, which are not
currently selected by an environment variable.

The registry in ``components/DDS/topics.h`` defines:

.. list-table:: Current messages
   :header-rows: 1

   * - DDS topic / ROS topic
     - Message
     - Current role
   * - ``rt/imu`` / ``/imu``
     - ``sensor_msgs/msg/Imu``
     - Writer and reader configured; Payload publishes debug IMU
   * - ``rt/motor_state`` / ``/motor_state``
     - ``payload_interfaces/msg/MotorState``
     - Writer and reader configured; Payload publishes encoder counts

The topic QoS is volatile, best effort, keep last 1. Reliable XRCE streams
used between client and Agent do not make those DDS topics reliable. Match reader
QoS when using ``ros2 topic echo`` as shown in the getting-started guide.

Learn DDS in small steps
------------------------

Start with the connection lifecycle, then follow one outgoing and one
incoming sample. The pages below explain the corresponding functions in the code.
You can return to this page for launch commands and topic names.

.. toctree::
   :maxdepth: 1

   connection
   topics
   sending-receiving
   development

The client has no explicit “ready” flag. Call ``init()`` before publishing, check
its logs, and keep servicing ``update()``. If initialization fails, the current
application does not stop the Payload loop or automatically reconnect. This is a
limitation of the implementation, not an assurance that later publishes are safe.

Add or change a message
-----------------------

1. Define its IDL under ``components/DDS/idl/<package>/msg/``. IDL describes the
   wire data structure, not application behavior.
2. Add the IDL to ``IDL_FILES`` and its generated C source to ``GENERATED_SOURCES``
   in the DDS component CMake file. Include any dependent IDL definitions.
3. Include the generated header in ``topics.h``; add a ``TopicId`` and matching
   ``make_topic<...>()`` entry to ``TYPED_TOPICS``. Enum indices and tuple order
   must agree; a static assertion checks this.
4. Match the DDS topic name, fully qualified DDS type name, direction, and the
   generated size/serialize/deserialize functions. Choose QoS deliberately.
5. Reconfigure and build each affected target using :doc:`../build-system`.
   Register a reader callback or add a typed publish at the intended call site.
6. Ensure the ROS-side interface definition agrees, then inspect with the Agent
   and ROS tools. A topic appearing in a list does not prove type compatibility.

Hardware transport boundary
---------------------------

Hardware builds select custom UART transport instead of UDP. However,
``uart_read`` and ``uart_write`` currently return zero and have TODO bodies;
``uart_open``/``uart_close`` are also placeholders. No working serial DDS setup
is established by the existing code, and the ESP entry point does not call DDS
initialization. Treat serial integration as implementation work, not a missing
Agent command.

Lookup: :doc:`../reference/communications`.
Sources: `DDS component <https://github.com/pennaerial/monorepo/tree/main/payload_controller/components/DDS>`_.
Further learning: `eProsima Micro XRCE-DDS architecture <https://micro-xrce-dds.docs.eprosima.com/en/stable/>`_.
