Payload Controller ESP
======================

Learn the C++ project in ``payload_controller/`` from first build to sensor fusion
and motor control. **ESP** refers to the ESP32-S3 microcontroller; the same project
also builds a program for your computer so you can learn with a simulated payload first.

Two build targets: simulation and hardware
------------------------------------------

A **build target** tells the tools which machine to make a program for:

* ``linux`` builds a program for a Linux computer. In this project it is the
  **simulation target**: it normally reads simulated sensors from Gazebo, a robot
  simulator, and sends commands back to that simulator. **Linux itself is an
  operating system**, such as Ubuntu, not a synonym for simulation in general.
* ``esp32s3`` builds **firmware**, the program installed on the physical ESP32-S3
  board. A microcontroller is a small computer that reads devices and controls
  electrical outputs. Its hardware features are still being connected; this
  target does not yet run the same complete loop as the simulation target.

When these guides say “Linux path” or “Linux build,” they mean the first target.
**SITL** means software-in-the-loop: running controller code with simulated
sensors and motors. **ESP-IDF** is Espressif's development framework for building
and running the board's firmware. **ROS** is a set of tools and libraries that
lets robot programs exchange data and coordinate work.

For an unfamiliar word, use the :doc:`glossary`. Each guide also introduces the
terms needed for its next step.

Start with :doc:`overview` to see how the pieces interact, then :doc:`getting-started`
to run them. Follow the pages in order for an introduction, or jump to
:doc:`reference/index` for function signatures. This is a different project from
the Python ROS package at ``ros_ws/src/sae/payload``.

.. toctree::
   :maxdepth: 3

   overview
   glossary
   getting-started
   project-organization
   components
   build-system
   dds_client/index
   sensors/index
   payload
   runtime-helpers
   simulation
   estimation/index
   control/index
   development-tools
   code-map
   reference/index
