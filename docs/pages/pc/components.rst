:doc-priority: key
:key-reason: Read before adding a component or changing its public/private dependencies.
:human-verified: true
:verified-by: User (review confirmed in chat)
:verified-on: 2026-11-21

Components and Dependencies
===========================

ESP-IDF builds the project's components and links them into one application.
Start with :doc:`project-organization` to locate the component that owns a feature.
This page explains how to declare its sources and dependencies.

Read a component declaration
----------------------------

The motor component selects ``motor_sitl.cpp`` for Linux and ``DRV8833.cpp`` for
hardware. Both provide ``Motor::instance(Side)``; compiling both would define the
same factory twice. Its common header lets Payload use either implementation.

The declaration below registers ``my_sensor.cpp`` and exports the current
folder as a public header search path through ``INCLUDE_DIRS``. Adding a new
``.cpp`` file to the folder also requires adding it to ``SRCS``.

In this example, you are editing **the my_sensor component's own**
``CMakeLists.txt``. It needs ``imu`` and ``esp_driver_i2c`` for different reasons:

.. code-block:: cmake

   # components/my_sensor/CMakeLists.txt
   idf_component_register(
       SRCS "my_sensor.cpp"
       INCLUDE_DIRS "."
       REQUIRES imu
       PRIV_REQUIRES esp_driver_i2c
   )

* Use ``REQUIRES imu`` if **my_sensor's public header**, ``my_sensor.hpp``,
  includes ``imu.hpp`` or exposes an IMU type. Both my_sensor itself and any
  component including that public header need access to the IMU headers.
  The public dependency makes those headers available to downstream consumers too.
* Use ``PRIV_REQUIRES esp_driver_i2c`` if the I2C include is **only in
  my_sensor.cpp**. The my_sensor component still needs the I2C dependency to
  compile its implementation, but a consumer of ``my_sensor.hpp`` does not need
  I2C headers through my_sensor's public interface.

Now consider a **different component**, such as ``payload``, that uses my_sensor.
In ``components/payload/CMakeLists.txt``, declare ``my_sensor`` as a dependency.
Use ``REQUIRES my_sensor`` if payload's public headers expose it, or
``PRIV_REQUIRES my_sensor`` if only payload's implementation uses it. Make that
choice from the perspective of the component whose CMake file you are editing.

A dependency being private does not remove it from the build; it limits what is
passed through the component's public interface. If payload directly includes
``imu.hpp`` or an I2C header for its own work, declare that dependency in payload
as well instead of depending on an unrelated component to supply it indirectly.

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
