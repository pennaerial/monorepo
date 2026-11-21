:doc-status: evolving
:status-reason: Hardware commands currently exercise IMU bring-up; integrated hardware control and wiring validation are unfinished.

Getting Started with Payload Controller
=======================================

Goal: build both targets, run the computer-hosted simulation with Gazebo, and understand
which executable and configuration you are using. Read :doc:`overview` first.
All commands below use Bash on the project's Ubuntu 24.04 environment.

Names used in the commands
--------------------------

``linux`` is this project's simulation build target; ``esp32s3`` is its physical
board target. **Gazebo** simulates the body, wheels, and sensors. **ESP-IDF** is the
board's build/runtime framework, and **EIM** installs it and its compiler tools.
A **toolchain** is the compiler and related tools needed to build for a machine.
The **DDS Agent** is a separate program that passes the controller's messages to
other robot programs. The parent :doc:`index` explains the two targets, and the
:doc:`glossary` is a quick lookup for these names.

Prepare the tools once
----------------------

Follow :doc:`../installation/ubuntu` for the existing installation instructions.
For payload work you need the repository and submodules, native C/C++ tools,
Gazebo Harmonic (``gz-transport13`` / ``gz-msgs10``), ESP-IDF **6.0.2**, and the DDS
tools. ROS 2 Jazzy is needed for the optional ROS inspection commands below.
You do not need to launch PX4 or a ROS mission for standalone payload simulation.

From the repository root, after installing EIM and the prerequisites in the setup guide:

.. code-block:: bash

   source dev_env.sh
   git submodule update --init --recursive
   eim install --config payload_controller/eim_config.toml

The existing :doc:`dependency installation guide <../installation/ubuntu>` covers
``Dependencies/build_all.sh``. That script builds and installs the XRCE Client,
Agent, and generator; it uses sudo for installation. Its Gradle step needs the
Java version described in that guide. The firmware CMake separately builds its
own target-specific Micro-CDR and XRCE Client libraries from those sources.

Prepare each terminal
---------------------

From the repository root:

.. code-block:: bash

   source dev_env.sh
   source "$HOME/.espressif/tools/activate_idf_v6.0.2.sh"
   export PATH="/usr/bin:$IDF_PATH/tools:$PATH"
   cd payload_controller
   idf.py --version
   gz sim --versions
   command -v MicroXRCEAgent

``dev_env.sh`` exports repository paths. The ESP activation script selects the
compiler tools and Python environment; it is unrelated to ROS's ``setup.bash``.
``PATH`` lists folders the shell searches for commands. The PATH line ensures
simulation builds find the host assembler (which translates assembly instructions
into machine code) and the actual ``idf.py`` script. Alternatively, configure direnv as described in the Ubuntu
guide; ``payload_controller/.envrc`` handles these tool paths automatically.
Repeat repository/toolchain setup in new terminals as needed.

Build and switch targets
------------------------

Run inside ``payload_controller/``:

.. code-block:: bash

   make linux
   make esp32s3

``linux`` builds ``build/linux/payload_controller.elf`` for your computer.
**ELF** is the executable-file format used for this program;
``esp32s3`` builds firmware in ``build/esp32s3/``. Each has its own **CMake cache**, saved build settings, and
``sdkconfig.linux`` or ``sdkconfig.esp32s3``. Switching is simply running the other
Make target; do not run a bare ``idf.py set-target`` over these separate build trees.

The last build also updates ``compile_commands.json`` for editor diagnostics.
Switch the editor back to an already-built target without rebuilding:

.. code-block:: bash

   make set_cc TARGET=linux
   # Or: make set_cc TARGET=esp32s3

This changes editor configuration only, not the executable running in another terminal.
See :doc:`build-system` for the underlying commands and cache behavior.

Run the standalone Gazebo simulation
------------------------------------

First terminal, after the dependency tools are installed:

.. code-block:: bash

   MicroXRCEAgent udp4 -p 7777

Second terminal, after preparing the payload environment above:

.. code-block:: bash

   make gz_payload
   # Without a GUI:
   # HEADLESS=1 make gz_payload
   # Select another existing world:
   # make gz_payload WORLD=sae

``gz_payload`` depends on ``linux``, so it builds first. The simulation script
starts a running world, waits for its create service, spawns the model as
``payload_0`` at Z=0.1 m, and runs the Linux executable with ``GZ_MODEL`` and
``GZ_WORLD`` set. It also supplies the resource paths and server system config.
The model name must match the sensor/joint conventions described in :doc:`sensors/index`.

Expect driver subscription messages, encoder readings, camera positions, and
``Kalman Filter`` state logs. The current speed targets are 7 rad/s left and
3 rad/s right, so asymmetric motion is intentional. The EKF initial position is
currently zero even though spawning uses Z=0.1 m; early camera corrections may
reflect that initial mismatch.

Press ``Ctrl+C`` to stop. The script cleans up Gazebo processes it started; a
pre-existing world is reused and left running. If you rerun against that world,
remove the previously spawned model or restart the world first: the script always
requests the same model name.

Inspect the connections
-----------------------

In another terminal:

.. code-block:: bash

   gz topic -l
   gz topic -e -t /world/default/model/payload_0/joint_state

For ROS inspection (ROS and any needed custom interfaces must be built/installed):

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   # From the repository root, if your ROS workspace has been built:
   source ros_ws/install/setup.bash
   export ROS_DOMAIN_ID=0
   ros2 topic list
   ros2 topic echo /imu --qos-reliability best_effort
   ros2 topic echo /motor_state --qos-reliability best_effort

A **DDS domain** separates groups of communicating programs; a **port** selects
which network service receives a packet. Domain 0 and port 7777 are fixed in the current client. Gazebo topic traffic
can work even if the Agent connection fails. See :doc:`dds_client/index` for that boundary.

Build, flash, and monitor the ESP
---------------------------------

With the appropriate board connected, from ``payload_controller/``:

.. code-block:: bash

   make esp32s3
   # Replace /dev/ttyACM0 with your board's actual port:
   idf.py -B build/esp32s3 -D SDKCONFIG=sdkconfig.esp32s3 -p /dev/ttyACM0 flash monitor

Exit the IDF monitor with ``Ctrl+]``. The default hardware IMU model is ICM20948;
to select the LSM6DSO backend, see :doc:`sensors/imu/hardware`. **The flashed
application currently runs IMU bring-up only**, as explained in :doc:`overview`.

If something fails
------------------

* ``idf.py`` missing: activate ESP-IDF or check direnv. ``IDF_PATH`` must be set.
* Missing ``PENNAIR_*`` variable: source ``dev_env.sh`` from the repository root.
* Missing generator or Java error: complete the DDS dependency build and check Java.
* Missing Gazebo CMake package: install Harmonic development dependencies, not just a GUI.
* DDS session error: start the Agent on UDP 7777; do not substitute PX4's common 8888 port.
* No samples: unpause simulation and compare actual topics, model name, world name,
  joint names, and sensor names with the :doc:`driver contracts <sensors/index>`.
* Assembler errors on Linux: check ``command -v as`` resolves to the host assembler;
  see the explanation in ``.envrc``.

Next: :doc:`project-organization`, then :doc:`components`.

Further learning: `Gazebo getting started <https://gazebosim.org/docs/harmonic/getstarted/>`_
and `ESP-IDF build guide <https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32s3/api-guides/build-system.html>`_.
