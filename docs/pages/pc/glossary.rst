Words and Tools Used in These Guides
====================================

Use this page for a quick lookup. Each learning guide also explains the terms
needed for its next step; you do not need to memorize this list before starting.
For the two build targets, begin with :doc:`index`.

.. glossary::
   :sorted:

   Linux target
      The ``linux`` build of this controller runs on a computer and normally
      connects to simulated sensors and motors. Linux itself is an operating
      system, such as the one used by Ubuntu; it does not mean simulation in general.

   ESP32-S3
      The physical microcontroller used by the firmware target ``esp32s3``.
      A microcontroller is a small computer with pins for reading/controlling electronics.

   Firmware
      The program installed on the microcontroller, rather than a desktop application.

   ESP-IDF
      Espressif's development framework: tools, libraries, and runtime support
      for making firmware for its chips. ``idf.py`` is its command-line build tool.

   EIM
      ESP-IDF Installation Manager, the tool used here to install ESP-IDF and
      the compiler tools selected by ``eim_config.toml``.

   SITL
      Software-in-the-loop: run controller code with simulated devices instead
      of physical ones. Here that is the usual use of the ``linux`` target.

   Gazebo
      A robot simulator that calculates motion, contact forces, and sensor outputs
      from a model. Harmonic is the Gazebo release used by this project.

   Gazebo Transport
      Gazebo's message library. Simulation drivers use it to receive sensor
      readings and send motor commands. It is a separate connection from DDS.

   ROS
      Robot Operating System: tools and libraries for building robot programs
      that exchange messages. Despite its name, it normally runs on an operating
      system such as Linux rather than replacing it.

   DDS
      Data Distribution Service, a system for exchanging named, typed messages
      between programs. “Typed” means the programs agree on the message fields.

   XRCE Agent
      A helper program that connects a small Micro XRCE-DDS client to the larger
      DDS network. Start it separately from the controller.

   UDP
      User Datagram Protocol: a network protocol that sends packets to an address
      and port. It does not by itself guarantee that each packet arrives.

   UART
      Universal Asynchronous Receiver-Transmitter: hardware for a serial byte
      connection between devices, usually using transmit, receive, and ground wires.
      Both ends must agree on settings such as the byte transmission rate.

   FreeRTOS
      A small real-time operating system used to schedule work on the controller.
      Its Linux port lets similar task code run on a computer. A requested delay
      does not guarantee an exact control-loop frequency.

   Task
      A scheduled piece of running work. Tasks/threads may take turns or run
      concurrently, so shared sensor data needs clear ownership and locking.

   Callback
      A function called when an event happens, such as a new sensor message arriving.

   Driver
      Code that reads or controls a device through a common interface.

   Backend
      One implementation of an interface. A simulated IMU and a physical IMU can
      supply the same operations while obtaining their readings in different ways.

   Cache
      Storage for the latest value. A sensor cache is not a history of every sample.
      A build cache is a different use of the word: saved configuration/build results.

   Mutex
      A lock that lets only one caller at a time access protected data, so a reader
      does not copy a half-updated sensor sample.

   CMake
      A tool that reads build descriptions and works out the sources and dependencies
      needed to make a program. Ninja then executes the resulting build commands.

   Make
      The command runner used by this project for named actions such as ``make linux``.
      Its Makefile is a short wrapper around ESP-IDF's build commands.

   API
      Application programming interface: the functions/types another part of the
      code can use. The API reference lists their exact declarations.

   IDL
      Interface Definition Language: the message schema used to describe fields
      from which this project's C message and encoding code is generated.

   Serialization
      Turning message fields into a defined sequence of bytes for sending.
      Deserialization turns received bytes back into local fields.

   QoS
      Quality of service: message delivery/history settings. The current DDS
      topics use best-effort delivery and keep only the latest sample.

   GPIO
      General-purpose input/output: a configurable chip pin for reading or driving
      an electrical signal. Pin assignments must match the actual board wiring.

   I2C
      A device bus using a data signal (SDA) and clock signal (SCL), plus shared
      power/ground connections. The controller selects a device using its address.

   PWM
      Pulse-width modulation: rapidly switch an electrical output and vary how
      much of each cycle it stays high. The motor driver turns a duty request
      into these waveforms; PWM duty is not wheel speed.

   LEDC
      ESP-IDF's name for a chip peripheral that generates PWM signals. This project
      uses it to drive the motor bridge inputs, even though its name comes from LED control.

   H-bridge
      A circuit that selects current direction through a motor and controls drive/braking.

   IMU
      Inertial measurement unit: a sensor combining a gyroscope for rotation rate
      and an accelerometer for specific force. It does not directly measure position.

   Frame
      A named set of coordinate axes. Body axes move with the payload; world axes
      stay fixed. A vector's numbers only make sense together with its frame and units.

   Quaternion
      Four numbers used together to represent a 3D rotation. A unit quaternion
      must remain normalized; adding arbitrary values to its coefficients is
      not the same as applying a small physical rotation.

   ES-EKF
      Error-state extended Kalman filter. It keeps a full motion estimate plus
      uncertainty in a small local error, then uses new measurements to correct
      that estimate. Also commonly called an ESKF. See :doc:`estimation/error-state-ekf`.

   Covariance
      A description of uncertainty. Diagonal entries hold variances (squared
      standard deviations); off-diagonal entries describe how errors are related.
      Covariance is part of the filter's model, not a guarantee of actual accuracy.

   Jacobian
      A table of local sensitivities: how a small change in one input/state changes
      an output. An EKF uses these to approximate nonlinear relationships nearby.

   Setpoint
      A desired value, such as the wheel speed a controller is asked to reach.

   PI and PID
      Feedback controllers named for proportional, integral, and derivative terms.
      The wheel controller currently uses PI: react to present speed error and
      accumulate persistent error. Its derivative gain is zero.

   Feedforward
      An estimate of required motor effort made from the desired motion before
      waiting for a measured error. Feedback then corrects what that estimate missed.
