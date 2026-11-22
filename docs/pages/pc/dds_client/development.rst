Changing and Debugging DDS
==========================

Use :doc:`connection` and :doc:`sending-receiving` to locate the stage that failed.
Keep the Agent, the application, and ROS inspection in separate terminals so their
logs stay readable. Launch commands are on :doc:`index`.

Add a message without losing track of the steps
-----------------------------------------------

1. Write or update the IDL schema under ``components/DDS/idl``. Agree on units,
   frame, field order, and meaning with its receiver.
2. Update both ``IDL_FILES`` and ``GENERATED_SOURCES`` in the DDS CMake file.
   The current build uses explicit lists, so placing a file in the folder is not enough.
3. Include the generated header from ``topics.h``. Add an enum ID and a matching
   typed registry entry, preserving zero-based tuple order.
4. Pass the generated size/serialize/deserialize functions for that same type to
   ``make_topic``. Use the exact DDS type name expected by the other participant.
5. Reconfigure and build from ``payload_controller/``:

   .. code-block:: bash

      idf.py -B build/linux -D SDKCONFIG=sdkconfig.linux reconfigure
      make linux
      make esp32s3

6. Publish at the intended application call site, or register a reader callback.
   Build/install the matching ROS interface if ROS needs to decode the message.
7. Run the Agent and application, inspect creation statuses, then check actual
   sample contents. Test values and units, not only whether a topic appears.

The generator command and output folder are printed by CMake. If code generation
fails, check the generator executable, Java setup, IDL includes, and the exported
``PENNAIR_XRCE_DDS_GEN_PATH``. Do not patch generated files in ``build``.

Troubleshoot from the connection outward
----------------------------------------

.. list-table:: Symptom to check
   :header-rows: 1
   :widths: 35 65

   * - Symptom
     - Start here
   * - UDP open succeeds, session fails
     - Agent running on UDP port 7777? Correct address? Fixed key reused by another client?
   * - Entity creation fails
     - Read each logged request status. Compare topic/type names and object identity with the registry.
   * - Topic appears but ROS gets no samples
     - Is Payload consuming fresh data? Does publish return true? Does update run? Does ROS have the matching type and best-effort QoS?
   * - Reliable output is full
     - Session not serviced often enough, missing Agent acknowledgments, or too much queued data. Check scheduling before increasing buffers.
   * - Message too large for one block
     - Compare serialized size plus 32 bytes with RELIABLE_STREAM_BLOCK_SIZE. The wrapper does not implement large-message handling.
   * - Incoming data logs an unknown reader
     - Check callback registration and ID/direction. A configured reader without an application callback is not enough.
   * - MotorState contains many zeros
     - Only encoder-count fields are currently filled by Payload; other schema fields are not live telemetry yet.
   * - No serial data on ESP
     - UART callbacks are TODOs, and the ESP entry point does not start DDS. This needs implementation before serial testing can succeed.

Limitations worth knowing before extending it
---------------------------------------------

The client uses a fixed address/key/domain, no explicit ready result, no automatic
reconnect, no general shutdown API, and a single-owner calling model. Hardware
``UartTransportConfig`` is declared locally during initialization; its lifetime
will also need review if future custom callbacks retain its address. Do not
mistake empty read/write callbacks returning zero for a working serial transport.

Sources: `DDS implementation and build files
<https://github.com/pennaerial/monorepo/tree/main/payload_controller/components/DDS>`_.
