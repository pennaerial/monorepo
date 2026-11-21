Getting Started
===============

Start here whether you are joining the club or moving to a different project.
You do not need robotics experience: first learn where things live, prepare your
computer and hardware, and make one small contribution before changing a robot.

How the systems fit together
----------------------------

A **sensor** measures the world. An **estimator** combines measurements to infer
motion. A **controller** compares a desired motion with measured motion and asks
an **actuator**, such as a motor, to do something. Simulation replaces physical
devices with software so you can observe this loop on your laptop.

The monorepo contains several interacting projects:

* **Payload Controller ESP** is the C++ embedded project in ``payload_controller/``.
  It has an ESP32-S3 target and a Linux software-in-the-loop (SITL) target.
* **ROS** is the workspace in ``ros_ws/``. ROS nodes exchange messages to coordinate
  vehicle behavior. Its mission and mode guides have their own section.
* **Gazebo** simulates bodies, joints, and sensors using models in ``gz-models/``.
  Payload SITL uses Gazebo Transport directly; ROS is useful for inspecting DDS
  output but is not required to start the standalone payload simulator.
* **DDS** connects the payload's small XRCE client to other DDS participants
  through an Agent. It is a separate connection from Gazebo Transport.

Your first path through the docs
--------------------------------

1. Read the :doc:`documentation map <organization>` and choose a project.
2. Complete :doc:`software setup <../installation/index>`. Follow only the
   toolchain sections required for your project before building it.
3. Read :doc:`hardware setup <../hardware/index>` before working with physical
   devices; simulation can be your first step without hardware.
4. Make your :doc:`first contribution <../tutorials/making-first-contrib>`.
5. For embedded work, continue to :doc:`Payload Controller ESP <../pc/index>`.

.. toctree::
   :maxdepth: 2

   organization
   Software Setup <../installation/index>
   Hardware Setup <../hardware/index>
   First Contribution <../tutorials/making-first-contrib>
   Development Workflow <../development-guide>
   Editing and Previewing Docs <../docs-contribution>
