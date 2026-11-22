How the Build Works
===================

Use :doc:`getting-started` for the short commands. This page explains what each
layer does, so a build failure can be traced to its owner.

From Make to an executable
--------------------------

.. mermaid::

   flowchart TB
       accTitle: Build pipeline
       accDescr: Make selects a target, IDF orchestrates configuration, CMake generates dependencies, and Ninja compiles and links target artifacts.
       make["make linux · make esp32s3"]
       idf["idf.py · target, SDKCONFIG, build directory"]
       cmake["CMake · components and dependencies"]
       ninja["Ninja · incremental compile and link"]
       linux(["Linux executable · build/linux"])
       esp(["ESP firmware · build/esp32s3"])
       make --> idf --> cmake --> ninja
       ninja --> linux & esp
       classDef compute fill:#ede9fe,stroke:#7c3aed,color:#3b0764
       classDef output fill:#dcfce7,stroke:#16a34a,color:#14532d
       class make,idf,cmake,ninja compute
       class linux,esp output

A Make rule has a **target**, optional **prerequisites**, and a tab-indented
**recipe** of shell commands. In ``gz_%: linux``, the pattern matches e.g.
``gz_payload``, first builds the ``linux`` prerequisite, and substitutes ``payload``
for ``$*`` in the simulation script command. ``WORLD ?= default`` supplies a
value only if one has not been provided; ``WORLD=sae`` overrides it.

The project Makefile is a command wrapper rather than the full C++ dependency
graph. Its target recipe checks for ``build/<target>/CMakeCache.txt``; on first
use it runs ``set-target`` with an explicit SDKCONFIG, adding ``--preview`` for
Linux. Every use then invokes ``idf.py ... build`` and updates the editor's
compile-command symlink. A normal incremental build recompiles changed sources.

Inspect the wrapper without running its recipes:

.. code-block:: bash

   make -n linux
   make -n gz_payload WORLD=default

The simulation wrapper is currently WIP and unreliable. The dry-run command
above is only for inspecting its recipe. To run the controller, use the manual
command and existing-world prerequisites in :doc:`getting-started`.

Source and configuration layers
-------------------------------

* Top-level ``CMakeLists.txt`` chooses C++20, includes IDF's project machinery,
  and adds nested driver component directories.
* Component ``CMakeLists.txt`` files select backend sources and link dependencies;
  see :doc:`components`.
* ``sdkconfig.defaults`` and ``sdkconfig.defaults.<target>`` seed settings when
  configuration is created. Existing ``sdkconfig.<target>`` files contain the
  resolved settings; editing a default does not necessarily override an existing setting.
* ``build/<target>/CMakeCache.txt`` holds cached configure variables. ``IMU_MODEL``
  is currently a CMake cache setting, not a menuconfig choice.
* ``build/<target>/compile_commands.json`` records compiler invocations. The
  root symlink chooses which target your editor understands.

Useful explicit commands
------------------------

From ``payload_controller/``, after building the chosen target at least once:

.. code-block:: bash

   idf.py -B build/linux -D SDKCONFIG=sdkconfig.linux reconfigure
   idf.py -B build/esp32s3 -D SDKCONFIG=sdkconfig.esp32s3 menuconfig
   make linux
   make esp32s3

Keep ``-B`` and ``SDKCONFIG`` paired when using IDF directly. ``make set_cc``
requires an existing compile-commands file; it does not build or set a chip target.
There is no project Make target named ``build``, ``sitl``, or ``flash`` here.

Generated code and debugging
----------------------------

DDS configuration runs ``components/DDS/generate.py`` on the explicit IDL list.
Generated ``.c``/``.h`` files live in that component's target build directory.
Edit IDL and the source lists, then reconfigure; do not edit generated serializers.
External DDS library configure/build/install logs live below that component's
``xrce_deps`` build directory and are printed on failure.

The repository has a Linux test-registration helper and a conversion test source,
but the top-level project does not currently wire a complete test-discovery path.
Do not assume ``make test`` or ``ctest`` exercises the controller. For this project,
check both target builds plus relevant simulation behavior until that integration
is completed. The ROS test workflow is a different system.

Further learning: `GNU Make rules <https://www.gnu.org/software/make/manual/html_node/Rule-Introduction.html>`_,
`CMake tutorial <https://cmake.org/cmake/help/latest/guide/tutorial/index.html>`_,
and the `ESP-IDF build guide <https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32s3/api-guides/build-system.html>`_.

How the external DDS libraries join the build
---------------------------------------------

The DDS CMake file builds Micro-CDR first, then the XRCE Client, using
``ExternalProject_Add``. Each target gets a private build/install folder inside
the DDS component's build directory. Compiler, toolchain, and install-prefix
settings are forwarded so an ESP build does not accidentally link a host library.
Linux also forwards its compiler flags because the Linux IDF toolchain file is
mostly a placeholder.

The libraries are static: their object code is linked into the application.
Imported CMake targets tell the application where the resulting archives and
headers are located; explicit dependencies ensure they are built first.
Linux enables UDP and disables serial/custom transport. Hardware disables UDP
and enables custom transport. TCP, discovery, shared memory, and multithread
profiles are disabled for both. This changes library features at compile time,
not merely a runtime port setting.

After locating Python, CMake runs the IDL generator, checks its exit result,
adds generated C files to the component, and exports the generated header folder.
Read :doc:`dds_client/topics` for the message-side explanation. The top-level
``eim_config.toml`` selects ESP-IDF/toolchain installation; it is not firmware
application configuration. ``idf_component.yml`` plus ``dependencies.lock``
control registry dependencies such as Eigen; see :doc:`components`.
