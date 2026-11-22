:human-verified: true
:verified-by: User (review confirmed in chat)
:verified-on: 2026-11-21
:doc-status: evolving
:status-reason: Simulation startup is WIP: make gz_payload is unreliable and the manual run requires an existing sae/payload_2 simulation. Hardware integration is also unfinished.

Getting Started with Payload Controller
=======================================

Goal: build both targets, run the computer-hosted simulation with Gazebo, and understand
which executable and configuration you are using. Read :doc:`overview` first.
All commands below use Bash on the project's Ubuntu 24.04 environment.

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
The PATH line selects the host assembler for simulation builds and makes
``idf.py`` available as a command. Alternatively, configure direnv as described in the Ubuntu
guide; ``payload_controller/.envrc`` handles these tool paths automatically.
Repeat repository/toolchain setup in new terminals as needed.

Build and switch targets
------------------------

Run inside ``payload_controller/``:

.. code-block:: bash

   make linux
   make esp32s3

``linux`` builds ``build/linux/payload_controller.elf`` for your computer;
``esp32s3`` builds firmware in ``build/esp32s3/``. Each has its own CMake cache and
``sdkconfig.linux`` or ``sdkconfig.esp32s3``. Switching is simply running the other
Make target; do not run a bare ``idf.py set-target`` over these separate build trees.

The last build also updates ``compile_commands.json`` for editor diagnostics.
Switch the editor back to an already-built target without rebuilding:

.. code-block:: bash

   make set_cc TARGET=linux
   # Or: make set_cc TARGET=esp32s3

This changes editor configuration only, not the executable running in another terminal.
See :doc:`build-system` for the underlying commands and cache behavior.

Run against an existing Gazebo simulation (WIP)
-----------------------------------------------

.. warning::

   **WIP:** ``make gz_payload`` is not working reliably. Use the manual command
   below for now. This workflow is also in progress: it assumes a running
   ``sae`` world with a model named ``payload_2``. The command starts only the
   controller; it does not start Gazebo or spawn the model.

First terminal, after installing the DDS tools:

.. code-block:: bash

   MicroXRCEAgent udp4 -p 7777

Second terminal, after preparing the payload environment, from ``payload_controller/``:

.. code-block:: bash

   make linux
   GZ_WORLD=sae GZ_MODEL=payload_2 ./build/linux/payload_controller.elf

The environment values select the world/model whose topics the drivers use.
Confirm those names match the running simulation. Expect driver subscription
messages, encoder readings, camera positions, and ``Kalman Filter`` state logs.
The current speed targets are 7 rad/s left and 3 rad/s right, so asymmetric motion
is intentional. The EKF currently starts at world position zero; a different
spawn pose can cause early camera corrections.

Press ``Ctrl+C`` to stop the controller. Gazebo remains running independently.
See :doc:`simulation` for the launch wrapper's intended behavior and current limits.

Inspect the connections
-----------------------

In another terminal:

.. code-block:: bash

   gz topic -l
   gz topic -e -t /world/sae/model/payload_2/joint_state

For ROS inspection (ROS and any needed custom interfaces must be built/installed):

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   # From the repository root, if your ROS workspace has been built:
   source ros_ws/install/setup.bash
   export ROS_DOMAIN_ID=0
   ros2 topic list
   ros2 topic echo /imu --qos-reliability best_effort
   ros2 topic echo /motor_state --qos-reliability best_effort

DDS domain 0 and Agent port 7777 are fixed in the current client. Gazebo topic traffic
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
