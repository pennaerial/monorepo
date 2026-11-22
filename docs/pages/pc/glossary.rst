Words and Tools Used in These Guides
====================================

Use this page to look up vocabulary used throughout the Payload Controller
guides. The other pages focus on how the code works; they explain a term again
only when it helps with a particular algorithm or decision. You do not need to
memorize this list before starting. For the learning path, begin with :doc:`index`.

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
      topics use best-effort delivery, volatile durability, and keep-last history
      with depth 1. See the entries for these policies below.

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

   Build target
      The platform selected when building. Here ``linux`` produces a computer executable
      and ``esp32s3`` produces firmware for the physical controller.

   Component
      An ESP-IDF build unit with its own sources, headers, and dependency declarations
      in a ``CMakeLists.txt`` file.

   Library
      Reusable code linked into a program or included through headers. A header-only
      library, such as Eigen, has no separate runtime process.

   Header
      A C/C++ file, usually ending in ``.h`` or ``.hpp``, that declares types and
      functions. Public headers form the interface offered to other components.

   Compiler and linker
      The compiler translates source into object code. The linker combines that code and
      its libraries into the final executable or firmware.

   Assembler
      The tool that translates assembly instructions into object code; it is normally
      invoked by the compiler toolchain.

   PATH
      The shell setting listing directories to search for commands. Sourcing the
      development environment adds the required tools to it.

   ELF
      Executable and Linkable Format, the file format used by the built
      ``payload_controller.elf``. The Linux version can be run on the host computer.

   Interface
      The operations and data a piece of code offers to callers, separate from how it
      implements them.

   Sensor
      A device or simulated source that measures a quantity, such as rotation rate or
      wheel motion.

   Actuator
      A device that acts on the physical system. Here the motors apply forces through
      the wheels.

   Angular speed
      Rotation rate, usually in radians per second (rad/s) in this project. A full turn
      is 2π radians.

   Duty
      The fraction of a PWM cycle spent high. The motor API uses a signed normalized
      request from -1 to +1 to express direction and effort.

   Bias
      A persistent offset in a measurement. For example, a stationary gyroscope can
      report a small nonzero rate.

   Noise
      Unwanted variation in a measurement or process. The estimator models its
      uncertainty rather than assuming each reading is exact.

   Pose
      Position and orientation together. A position-only camera message does not supply
      a full pose.

   Ground truth
      The simulator's underlying state, used as a reference for comparison or to
      generate simulated measurements.

   SDF
      Simulation Description Format, the files that describe Gazebo worlds, models,
      links, joints, and sensors.

   FreeRTOS Linux port
      The implementation that lets FreeRTOS tasks run on Linux. This project uses
      SIGALRM for scheduling; it is still a host program, not ESP hardware.

   SIGALRM
      A Unix signal used by this FreeRTOS Linux port to trigger scheduling. Gazebo
      worker threads must block it to avoid interfering with their locks.

   Thread
      An independently scheduled sequence of work within a process. Gazebo callbacks and
      controller tasks can access data at different times.

   Static allocation
      Reserving storage ahead of time rather than requesting it dynamically while the
      program runs.

   RAII
      Resource Acquisition Is Initialization: a C++ pattern that acquires a resource in
      an object's constructor and releases it in its destructor. Lock guards use it to
      release mutexes automatically.

   Function pointer
      A value identifying a function to call. The DDS subscription registry uses
      callbacks to handle incoming messages.

   Template parameter
      A compile-time argument to C++ code, such as a message type or matrix size.
      Different arguments can produce different compiled implementations.

   Tuple
      A fixed-size collection whose elements can have different types. The DDS registry
      groups topic descriptions this way.

   CDR
      Common Data Representation, the byte encoding used for DDS messages. Micro-CDR
      supplies serialization helpers for the embedded client.

   Topic
      A named message channel with an agreed message type. Publishers and subscribers
      must use compatible names, types, and settings.

   DataWriter and DataReader
      DDS entities that send and receive topic samples. A writer belongs to a publisher;
      a reader belongs to a subscriber.

   Participant
      A DDS entity representing an application's participation in a DDS domain. The XRCE
      Agent creates it on behalf of this client.

   Publisher and subscriber
      DDS entities that group writers and readers respectively. More generally,
      publishing means sending samples and subscribing means registering to receive
      them.

   XRCE session
      The connection state between a client and its Agent, including identifiers and
      message streams.

   Stream
      An XRCE channel for moving protocol messages between client and Agent. Its buffers
      and delivery mode are configured separately from a DDS topic.

   MTU
      Maximum transmission unit: the packet size limit used for a transport or buffer
      configuration.

   Socket
      An operating-system endpoint for network communication. The Linux client uses a
      UDP socket to contact the Agent.

   Port
      A number identifying a network endpoint on a host. UDP port 7777 is the Agent
      endpoint used in these instructions.

   DDS domain
      A numbered logical DDS network. Programs generally need matching domain IDs to
      discover each other; this is separate from the Agent's UDP port.

   Best effort
      A delivery policy that does not guarantee every sample arrives. Consumers must
      cope with missing updates.

   Volatile durability
      A DDS policy that does not retain historical samples for a subscriber that joins
      later.

   Keep last
      A DDS history policy retaining only a configured number of recent samples. Depth 1
      keeps the latest sample.

   Polling
      Checking repeatedly for available work or new data. The client must service its
      XRCE session for incoming callbacks to run.

   Encoder
      A sensor that measures shaft motion, often by counting pulses. The controller uses
      counts and angular speed for wheel feedback.

   Quadrature
      Two encoder signals offset in phase. Their order indicates direction, and their
      transitions can be counted to measure rotation.

   Interrupt
      An event that makes a processor run a handler promptly, such as when an encoder
      input changes.

   Standard deviation
      A measure of spread in a distribution, expressed in the same units as the
      quantity. Variance is its square.

   Random seed
      The starting value for a pseudo-random generator. Reusing it helps reproduce a
      simulated noise sequence.

   Lever arm
      The displacement between a sensor and the reference point whose motion is being
      estimated. Rotation can make those points move differently.

   Hz
      Hertz: events or cycles per second. A 10 Hz loop runs ten times per second when
      timing permits.

   Toolchain
      The compiler, assembler, linker, and related tools selected to build for
      a particular platform.

   Register
      A numbered location in a device for a configuration setting or measurement.
      Drivers read and write these locations over a bus such as I2C.

   Reliable stream
      An XRCE stream that tracks delivery between client and Agent. This does
      not by itself make the associated DDS topic's delivery policy reliable.

   Unity
      The C/C++ assertion framework used by the project's test helper.
