Software Setup
==============

.. rst-class:: lead

    Set up your laptop for monorepo development.

Monorepo is only supported on Ubuntu 24.04. For Windows users, we recommend you configure a Linux dual-boot
setup or WSL2 devcontainer, and for macOS, you must run Monorepo from a UTM virtual machine running Ubuntu 24.04.

Choose your project tools
-------------------------

The computer hosts editors and build tools; a toolchain turns source into an
executable for either Linux or an embedded chip. Shell setup scripts select the
right toolchain and export paths; run them in each new terminal as directed.

The Ubuntu guide covers the full monorepo stack. For Payload Controller ESP,
use its repository, Gazebo Harmonic, ESP-IDF, and DDS dependency instructions,
then follow :doc:`../pc/getting-started`. PX4 flight simulation and ROS missions
are separate workflows. For ROS work, complete ROS 2 and workspace setup as well.
Return to the :doc:`documentation map <../getting-started/organization>` when
choosing your next learning path.

.. toctree::
   :maxdepth: 2
   :caption: Setup Guides

   Ubuntu <ubuntu>
   macOS <macos_utm>
   Windows <wsl2_devcontainer>

   Upgrading an Existing Ubuntu Setup <ubuntu-update>
