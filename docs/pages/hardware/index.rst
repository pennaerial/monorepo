Hardware Setup
==============

.. rst-class:: lead

    Prepare physical devices and understand their role in the system.

How hardware connects
---------------------

A Raspberry Pi runs Linux/ROS services and connects through Ethernet/SSH for
configuration. The ESP32-S3 runs the separate C++ payload firmware and connects
to sensors/actuators through peripheral buses and pins. Gazebo replaces those
physical devices during SITL; you can start with simulation before acquiring hardware.

Use the Pi guides below for imaging/network access. For ESP toolchain, firmware,
and board-specific status, follow :doc:`../pc/getting-started` and the individual
:doc:`payload driver hardware pages <../pc/sensors/index>`. Their wiring assumptions
must match the board revision; several hardware paths are still in progress.

.. toctree::
   :maxdepth: 1
   :caption: Setup Guides

   Raspberry Pi Setup <raspberry-pi>
   Ethernet and SSH <ethernet>
   PX4 / Pi Integration Notes <../hardware-integration>
