:doc-status: evolving
:status-reason: The computer-hosted simulation runs the full loop. The physical board currently runs an IMU sensor test; several hardware drivers are unfinished.

System Overview
===============

A payload controller reads sensors, estimates motion, and controls two wheel
motors. Begin with the data flow before looking at individual classes.

Reading the diagram
-------------------

* **Backend / driver:** code that reads or controls a device. The simulation
  backend talks to Gazebo; the hardware backend talks to physical electronics.
* **IMU (inertial measurement unit):** a sensor that measures rotation rate and
  specific force, the acceleration-like signal that includes support against gravity.
* **Encoder:** a sensor that counts wheel or shaft rotation to estimate speed.
* **Cache:** storage for the latest reading. Several parts of the program can
  read it without asking the sensor to measure again.
* **EKF (extended Kalman filter):** the estimator that combines sensor readings
  to estimate position, velocity, and orientation.
* **PI + feedforward:** the speed controller. PI corrects the difference between
  desired and measured speed; feedforward estimates the effort needed in advance.
* **DDS (Data Distribution Service):** a system for sharing named, typed messages
  between programs. Here it sends debug readings to other robot software.
* **XRCE Agent:** a helper program that connects the small DDS client to the
  wider DDS network. **ROS** provides robot software tools that can read that data.
* **Motor duty:** a signed effort request from -1 to +1; **torque** is turning force.

These names describe different jobs: the EKF estimates motion, the controller
chooses motor effort, and DDS carries messages. Follow :doc:`glossary` for more terms.

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
read devices and drive an **H-bridge**, a circuit that controls motor direction and drive; the full hardware loop and some producers
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
On the ``linux`` simulation target it starts **Gazebo Transport**, the simulator's
message system for sensor readings and motor commands. It also starts the DDS
connection and initializes Payload at world position ``(0,0,0)``. It then
repeatedly services DDS and calls ``Payload::update()``.
**FreeRTOS** is the small operating system used to schedule tasks in the controller;
its Linux version lets those tasks run on your computer. The loop asks FreeRTOS
to wait 100 ms between passes. That delay alone corresponds to about ten passes
per second (10 Hz); DDS work and scheduling can make the loop slower. A **callback** is a function run when an event occurs;
sensor callbacks can receive new readings between passes through this loop.

On the physical ESP32-S3 board, the current entry point only obtains the IMU, calls ``start()``, then
calls ``update()`` in the loop. It does not initialize the full Payload, motors,
encoders, camera, or DDS. A successful hardware build does not establish feature
parity with Linux.

The simulation path has simulated IMU and encoders, torque-producing motors, and a
provisional noisy position camera. Hardware encoder acquisition, camera production,
and **UART** DDS transfer are still unimplemented. UART is a serial connection
that sends bytes over transmit/receive wires between electronic devices. See each :doc:`sensors/index`
page before choosing a bench task.

Words, units, and frames
------------------------

* **SITL:** software-in-the-loop; run application code against simulated devices.
* **Backend:** an implementation of the same driver interface for one environment.
* **Body frame:** axes attached to the payload; X is forward in the estimator.
* **World frame:** fixed room/simulation coordinates, with Z pointing up.
* **Angular speed:** radians/second; one revolution is ``2*pi`` radians.
* **Duty:** signed normalized motor request in ``[-1,1]``; not a speed in rad/s.
* **Bias:** a persistent measurement offset; **noise** varies between readings.

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
