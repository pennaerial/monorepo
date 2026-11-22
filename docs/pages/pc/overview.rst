:doc-priority: key
:key-reason: Start here for the sensor, estimation, control, and communication paths and their current hardware limits.
:human-verified: true
:verified-by: User (review confirmed in chat)
:verified-on: 2026-11-21
:doc-status: evolving
:status-reason: The computer-hosted simulation runs the full loop. The physical board currently runs an IMU sensor test; several hardware drivers are unfinished.

System Overview
===============

A payload controller reads sensors, estimates motion, and controls two wheel
motors. Begin with the data flow before looking at individual classes.

.. mermaid::

   flowchart TB
       accTitle: Payload Controller data flow
       accDescr: Simulated or physical sensor backends feed shared caches. Estimation, debug publishing, and speed control read them independently. Only speed control commands the motors.
       world(["Payload motion · simulation / real hardware"])
       subgraph input["1 · Sensor backends"]
           imu["IMU"]
           enc["Encoders"]
           cam["Position source"]
           cache[("Latest-sample caches")]
           imu --> cache
           enc --> cache
           cam --> cache
       end
       subgraph app["2 · Payload update"]
           est["EKF"]
           debug["DDS debug"]
           ctrl["PI + feedforward"]
       end
       world --> imu & enc & cam
       cache --> est & debug
       cache -->|"wheel speed"| ctrl
       debug --> agent["XRCE Agent / ROS"]
       ctrl -->|"duty −1 to +1"| motor["Motor backend"]
       motor -->|"torque"| world
       classDef sensor fill:#e0f2fe,stroke:#0284c7,color:#0c4a6e
       classDef compute fill:#ede9fe,stroke:#7c3aed,color:#3b0764
       classDef comm fill:#dcfce7,stroke:#16a34a,color:#14532d
       classDef actuator fill:#ffedd5,stroke:#ea580c,color:#7c2d12
       classDef physical fill:#f1f5f9,stroke:#64748b,color:#334155
       class imu,enc,cam,cache sensor
       class world physical
       class est,ctrl compute
       class debug,agent comm
       class motor actuator
       style input fill:#f8fafc,stroke:#cbd5e1,color:#334155
       style app fill:#f8fafc,stroke:#cbd5e1,color:#334155

The diagram shows the shared design for simulation and physical hardware.
SITL backends read Gazebo data and apply simulated torque. Hardware backends
read devices and drive an H-bridge; the full hardware loop and some producers
are still unfinished, as described below. “Position source” currently means
the simulated camera stand-in, not a completed hardware camera pipeline.

Diagram key: blue is measurement, purple is computation, green is communication,
orange is actuation, and gray is the physical system. Drag to pan, scroll to zoom, or use the fullscreen button.

The estimator and speed controller are separate consumers of sensor data.
**The current speed controller uses encoder velocity directly, not EKF position.**
There is no implemented position-to-speed outer control loop here.

What runs today
---------------

``main/payload_controller.cpp`` defines ``app_main()``, the application entry point.
On the ``linux`` simulation target it starts Gazebo Transport and the DDS
connection and initializes Payload at world position ``(0,0,0)``. It then
repeatedly services DDS and calls ``Payload::update()``.
The loop requests a 100 ms FreeRTOS delay between passes: at most about 10 Hz
before DDS work and scheduling add time. Sensor callbacks may receive readings
between passes through this loop.

On the physical ESP32-S3 board, the current entry point only obtains the IMU, calls ``start()``, then
calls ``update()`` in the loop. It does not initialize the full Payload, motors,
encoders, camera, or DDS. A successful hardware build does not establish feature
parity with Linux.

The simulation path has simulated IMU and encoders, torque-producing motors, and a
provisional noisy position camera. Hardware encoder acquisition, camera production,
and UART DDS transfer are still unimplemented. See :doc:`sensors/index` before
choosing a bench task.

Next steps
----------

Run :doc:`getting-started`, locate files with :doc:`project-organization`, then
trace :doc:`payload`. For a function lookup, use :doc:`reference/index`.

Source: `application entry point <https://github.com/pennaerial/monorepo/blob/main/payload_controller/main/payload_controller.cpp>`_.

More detailed code paths
------------------------

For the networking steps, follow :doc:`dds_client/connection`,
:doc:`dds_client/topics`, and :doc:`dds_client/sending-receiving` in order.
For cross-cutting code, read :doc:`runtime-helpers` and :doc:`simulation`.
The :doc:`code-map` links every tracked project source/build file to its guide.
