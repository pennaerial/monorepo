Components and Dependencies
===========================

A component groups source files, public headers, and dependencies into a library.
A **library** is reusable compiled code. **ESP-IDF**, the board development
framework, builds those libraries and **links** them into one application: it
joins the compiled pieces so their function calls can reach each other.
**CMake** reads each component's build instructions. A **header** declares the
interface other components can use; an implementation file supplies the code. Start with
:doc:`project-organization` to find the component that owns a feature.

Read a component declaration
----------------------------

The motor component selects ``motor_sitl.cpp`` for Linux and ``DRV8833.cpp`` for
hardware. Both provide ``Motor::instance(Side)``; compiling both would define the
same factory (the function returning the selected driver) twice. Its common header lets Payload use either implementation.

.. code-block:: cmake

   idf_component_register(
       SRCS "my_sensor.cpp"
       INCLUDE_DIRS "."
       REQUIRES util
       PRIV_REQUIRES esp_driver_i2c
   )

* ``SRCS`` lists C++ source files to compile. Adding a ``.cpp`` file to the
  directory alone does not add it to this explicit source list.
* ``INCLUDE_DIRS`` exports public header search paths.
* ``REQUIRES`` expresses public dependencies, needed by a consumer reading your headers.
* ``PRIV_REQUIRES`` expresses dependencies used inside your implementation.

For example, if ``my_sensor.hpp`` includes ``imu.hpp``, expose ``imu`` with
``REQUIRES``. If only ``my_sensor.cpp`` uses I2C, keep ``esp_driver_i2c`` private.
Declare actual direct dependencies instead of relying on headers pulled in indirectly by another component.

Add or change a component
-------------------------

1. Decide its contract: measured quantity, units, frame, freshness, and owner.
   Separate device I/O from estimator/control logic.
2. Create ``components/my_sensor/`` (or ``components/drivers/my_sensor/`` for a
   device), a public header, implementation, and ``CMakeLists.txt``.
3. Register sources and dependencies as above. Select Linux and hardware source
   files with ``if(IDF_TARGET STREQUAL "linux")`` where necessary.
4. Add the component to its consumer's ``REQUIRES`` or ``PRIV_REQUIRES``.
   On Linux the project starts from ``main`` and follows dependencies; an unused
   component may not be built simply because it exists.
5. Add initialization and sample consumption at the appropriate layer. Follow
   the :doc:`cache contract <sensors/index>` for asynchronous producers.
6. From ``payload_controller/``, run ``make linux`` and ``make esp32s3``. Run the
   relevant simulation/bench checks; document hardware work that is still a stub.
7. Add a subsystem overview and API entry under :doc:`reference/index`.

After changing CMake or manifests, explicitly reconfigure when diagnosing stale
configuration. Use the same target directory and SDKCONFIG every time:

.. code-block:: bash

   idf.py -B build/linux -D SDKCONFIG=sdkconfig.linux reconfigure
   make linux

Eigen and managed dependencies
------------------------------

Eigen provides vector, matrix, and quaternion operations used by the EKF.
``Eigen::Vector3f`` is a three-element float vector; ``Matrix<float,15,15>`` is a
fixed-size covariance matrix; ``Quaternionf`` stores orientation without Euler-angle
singularities. A matrix block such as ``block<3,3>(0,3)`` selects a 3-by-3 submatrix.

``components/payload/idf_component.yml`` declares ``espressif/eigen`` with
``public: true`` because public headers such as ``ekf.hpp`` expose Eigen types.
The manifest currently allows any version (``'*'``), while ``dependencies.lock``
records the resolved version. The IDF Component Manager populates
``managed_components/``. Edit the manifest and review lockfile changes when
changing a dependency; do not patch downloaded managed files as the source of truth.

Eigen is a header-only library: its templates are compiled where used. There is
no standalone Eigen runtime process. Fixed-size matrices make dimensions visible;
the current correction code still uses dynamic ``MatrixXf`` temporaries and an
LDLT solve, discussed in :doc:`estimation/ekf`.

Other dependency mechanisms
---------------------------

DDS builds Micro-CDR and Micro XRCE-DDS Client using ``ExternalProject_Add`` from
repository dependency sources, with different transport profiles per target.
Gazebo packages are found by CMake on Linux through ``cmake/gz_deps.cmake``.
These mechanisms are separate from registry-managed Eigen.

Further learning: `ESP-IDF build system <https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32s3/api-guides/build-system.html>`_
and `Eigen introduction <https://eigen.tuxfamily.org/dox/GettingStarted.html>`_.
