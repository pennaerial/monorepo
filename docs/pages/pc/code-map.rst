Code Reading Map
================

Use this index when a filename is unfamiliar. Use :doc:`glossary` when the term
itself is unfamiliar. It covers the tracked source,
headers, scripts, schemas, and build/configuration files in root
``payload_controller/``. Generated build output and downloaded dependencies are
not maintained source files. Header/implementation pairs share a guide because
one declares the interface and the other implements its behavior.

Start at :doc:`overview` for the runtime flow. For DDS, read
:doc:`dds_client/connection`, :doc:`dds_client/topics`, and
:doc:`dds_client/sending-receiving` together; the table links to the main guide
for each file. Human review is still required before treating these guides as verified.

.. list-table:: File to explanation
   :header-rows: 1
   :widths: 46 24 30

   * - Source (relative to payload_controller)
     - Guide
     - What it explains
   * - `.clang-tidy <https://github.com/pennaerial/monorepo/blob/main/payload_controller/.clang-tidy>`_
     - :doc:`development-tools`
     - Static-analysis configuration and command/filter
   * - `.envrc <https://github.com/pennaerial/monorepo/blob/main/payload_controller/.envrc>`_
     - :doc:`getting-started`
     - ESP-IDF installation and shell tool paths
   * - `.gitignore <https://github.com/pennaerial/monorepo/blob/main/payload_controller/.gitignore>`_
     - :doc:`build-system`
     - Generated build/config files kept out of source control
   * - `CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/CMakeLists.txt>`_
     - :doc:`build-system`
     - Source/backend selection and component dependencies
   * - `Doxyfile <https://github.com/pennaerial/monorepo/blob/main/payload_controller/Doxyfile>`_
     - :doc:`reference/index`
     - C++ source-to-XML reference generation
   * - `Makefile <https://github.com/pennaerial/monorepo/blob/main/payload_controller/Makefile>`_
     - :doc:`build-system`
     - Build commands, target configuration, Gazebo linkage
   * - `cmake/gz_deps.cmake <https://github.com/pennaerial/monorepo/blob/main/payload_controller/cmake/gz_deps.cmake>`_
     - :doc:`build-system`
     - Build commands, target configuration, Gazebo linkage
   * - `cmake/tests.cmake <https://github.com/pennaerial/monorepo/blob/main/payload_controller/cmake/tests.cmake>`_
     - :doc:`development-tools`
     - Test helper/placeholder; current coverage limits
   * - `components/DDS/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/CMakeLists.txt>`_
     - :doc:`components`
     - Source/backend selection and component dependencies
   * - `components/DDS/dds_client.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/dds_client.cpp>`_
     - :doc:`dds_client/sending-receiving`
     - Publish/session/callback path; also read DDS connection guide
   * - `components/DDS/dds_client.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/dds_client.hpp>`_
     - :doc:`dds_client/sending-receiving`
     - Publish/session/callback path; also read DDS connection guide
   * - `components/DDS/generate.py <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/generate.py>`_
     - :doc:`dds_client/topics`
     - Schema, registry, generated types and byte encoding
   * - `components/DDS/idl/builtin_interfaces/msg/Time.idl <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/idl/builtin_interfaces/msg/Time.idl>`_
     - :doc:`dds_client/topics`
     - Schema, registry, generated types and byte encoding
   * - `components/DDS/idl/geometry_msgs/msg/Quaternion.idl <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/idl/geometry_msgs/msg/Quaternion.idl>`_
     - :doc:`dds_client/topics`
     - Schema, registry, generated types and byte encoding
   * - `components/DDS/idl/geometry_msgs/msg/Vector3.idl <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/idl/geometry_msgs/msg/Vector3.idl>`_
     - :doc:`dds_client/topics`
     - Schema, registry, generated types and byte encoding
   * - `components/DDS/idl/payload_interfaces/msg/MotorState.idl <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/idl/payload_interfaces/msg/MotorState.idl>`_
     - :doc:`dds_client/topics`
     - Schema, registry, generated types and byte encoding
   * - `components/DDS/idl/sensor_msgs/msg/Imu.idl <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/idl/sensor_msgs/msg/Imu.idl>`_
     - :doc:`dds_client/topics`
     - Schema, registry, generated types and byte encoding
   * - `components/DDS/idl/std_msgs/msg/Header.idl <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/idl/std_msgs/msg/Header.idl>`_
     - :doc:`dds_client/topics`
     - Schema, registry, generated types and byte encoding
   * - `components/DDS/topics.h <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/topics.h>`_
     - :doc:`dds_client/topics`
     - Schema, registry, generated types and byte encoding
   * - `components/DDS/uart_transport.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/uart_transport.cpp>`_
     - :doc:`dds_client/development`
     - Custom UART contract and unimplemented byte transfer
   * - `components/DDS/uart_transport.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/DDS/uart_transport.hpp>`_
     - :doc:`dds_client/development`
     - Custom UART contract and unimplemented byte transfer
   * - `components/drivers/camera/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/camera/CMakeLists.txt>`_
     - :doc:`components`
     - Source/backend selection and component dependencies
   * - `components/drivers/camera/camera.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/camera/camera.cpp>`_
     - :doc:`sensors/camera/index`
     - Common driver interface and cache contract
   * - `components/drivers/camera/camera.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/camera/camera.hpp>`_
     - :doc:`sensors/camera/index`
     - Common driver interface and cache contract
   * - `components/drivers/camera/camera_sitl.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/camera/camera_sitl.cpp>`_
     - :doc:`sensors/camera/sitl`
     - Backend behavior
   * - `components/drivers/encoder/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/CMakeLists.txt>`_
     - :doc:`components`
     - Source/backend selection and component dependencies
   * - `components/drivers/encoder/encoder.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/encoder.cpp>`_
     - :doc:`sensors/encoders/index`
     - Common driver interface and cache contract
   * - `components/drivers/encoder/encoder.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/encoder.hpp>`_
     - :doc:`sensors/encoders/index`
     - Common driver interface and cache contract
   * - `components/drivers/encoder/encoder_hw.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/encoder_hw.cpp>`_
     - :doc:`sensors/encoders/hardware`
     - Backend behavior
   * - `components/drivers/encoder/encoder_hw.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/encoder_hw.hpp>`_
     - :doc:`sensors/encoders/hardware`
     - Backend behavior
   * - `components/drivers/encoder/encoder_sitl.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/encoder_sitl.cpp>`_
     - :doc:`sensors/encoders/sitl`
     - Backend behavior
   * - `components/drivers/encoder/encoder_sitl.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/encoder_sitl.hpp>`_
     - :doc:`sensors/encoders/sitl`
     - Backend behavior
   * - `components/drivers/imu/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/CMakeLists.txt>`_
     - :doc:`components`
     - Source/backend selection and component dependencies
   * - `components/drivers/imu/icm20948.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/icm20948.cpp>`_
     - :doc:`sensors/imu/hardware`
     - Backend behavior
   * - `components/drivers/imu/icm20948.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/icm20948.hpp>`_
     - :doc:`sensors/imu/hardware`
     - Backend behavior
   * - `components/drivers/imu/imu.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/imu.cpp>`_
     - :doc:`sensors/imu/index`
     - Common driver interface and cache contract
   * - `components/drivers/imu/imu.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/imu.hpp>`_
     - :doc:`sensors/imu/index`
     - Common driver interface and cache contract
   * - `components/drivers/imu/imu_sitl.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/imu_sitl.cpp>`_
     - :doc:`sensors/imu/sitl`
     - Backend behavior
   * - `components/drivers/imu/imu_sitl.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/imu_sitl.hpp>`_
     - :doc:`sensors/imu/sitl`
     - Backend behavior
   * - `components/drivers/imu/lsm6dso.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/lsm6dso.cpp>`_
     - :doc:`sensors/imu/hardware`
     - Backend behavior
   * - `components/drivers/imu/lsm6dso.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/imu/lsm6dso.hpp>`_
     - :doc:`sensors/imu/hardware`
     - Backend behavior
   * - `components/drivers/motor/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/CMakeLists.txt>`_
     - :doc:`components`
     - Source/backend selection and component dependencies
   * - `components/drivers/motor/DRV8833.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/DRV8833.cpp>`_
     - :doc:`sensors/motors/hardware`
     - Backend behavior
   * - `components/drivers/motor/DRV8833.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/DRV8833.hpp>`_
     - :doc:`sensors/motors/hardware`
     - Backend behavior
   * - `components/drivers/motor/motor.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/motor.cpp>`_
     - :doc:`sensors/motors/index`
     - Common driver interface and cache contract
   * - `components/drivers/motor/motor.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/motor.hpp>`_
     - :doc:`sensors/motors/index`
     - Common driver interface and cache contract
   * - `components/drivers/motor/motor_sitl.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/motor_sitl.cpp>`_
     - :doc:`sensors/motors/sitl`
     - Backend behavior
   * - `components/drivers/motor/motor_sitl.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/motor_sitl.hpp>`_
     - :doc:`sensors/motors/sitl`
     - Backend behavior
   * - `components/payload/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/payload/CMakeLists.txt>`_
     - :doc:`components`
     - Source/backend selection and component dependencies
   * - `components/payload/ekf.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/payload/ekf.cpp>`_
     - :doc:`estimation/ekf`
     - State, prediction, sensor corrections and covariance update
   * - `components/payload/ekf.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/payload/ekf.hpp>`_
     - :doc:`estimation/ekf`
     - State, prediction, sensor corrections and covariance update
   * - `components/payload/idf_component.yml <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/payload/idf_component.yml>`_
     - :doc:`components`
     - Managed Eigen dependency and resolved versions
   * - `components/payload/payload.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/payload/payload.cpp>`_
     - :doc:`payload`
     - Sensor collection, estimator inputs, debug and motor updates
   * - `components/payload/payload.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/payload/payload.hpp>`_
     - :doc:`payload`
     - Sensor collection, estimator inputs, debug and motor updates
   * - `components/sitl/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/sitl/CMakeLists.txt>`_
     - :doc:`components`
     - Source/backend selection and component dependencies
   * - `components/sitl/convert.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/sitl/convert.cpp>`_
     - :doc:`simulation`
     - Simulation startup, transport threads or message conversion
   * - `components/sitl/convert.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/sitl/convert.hpp>`_
     - :doc:`simulation`
     - Simulation startup, transport threads or message conversion
   * - `components/sitl/sitl_runtime.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/sitl/sitl_runtime.cpp>`_
     - :doc:`simulation`
     - Simulation startup, transport threads or message conversion
   * - `components/sitl/sitl_runtime.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/sitl/sitl_runtime.hpp>`_
     - :doc:`simulation`
     - Simulation startup, transport threads or message conversion
   * - `components/sitl/test/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/sitl/test/CMakeLists.txt>`_
     - :doc:`development-tools`
     - Test helper/placeholder; current coverage limits
   * - `components/sitl/test/test_convert.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/sitl/test/test_convert.cpp>`_
     - :doc:`development-tools`
     - Test helper/placeholder; current coverage limits
   * - `components/util/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/CMakeLists.txt>`_
     - :doc:`components`
     - Source/backend selection and component dependencies
   * - `components/util/feedforward.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/feedforward.cpp>`_
     - :doc:`control/feedforward`
     - Control algorithm
   * - `components/util/feedforward.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/feedforward.hpp>`_
     - :doc:`control/feedforward`
     - Control algorithm
   * - `components/util/log_rate_limiter.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/log_rate_limiter.hpp>`_
     - :doc:`runtime-helpers`
     - Shared ownership, locking, timing, or side selection
   * - `components/util/payload_corner.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/payload_corner.hpp>`_
     - :doc:`runtime-helpers`
     - Shared ownership, locking, timing, or side selection
   * - `components/util/pid.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/pid.cpp>`_
     - :doc:`control/pid`
     - Control algorithm
   * - `components/util/pid.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/pid.hpp>`_
     - :doc:`control/pid`
     - Control algorithm
   * - `components/util/pwmcontroller.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/pwmcontroller.cpp>`_
     - :doc:`control/pwmcontroller`
     - Control algorithm
   * - `components/util/pwmcontroller.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/pwmcontroller.hpp>`_
     - :doc:`control/pwmcontroller`
     - Control algorithm
   * - `components/util/static_mutex.hpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/static_mutex.hpp>`_
     - :doc:`runtime-helpers`
     - Shared ownership, locking, timing, or side selection
   * - `dependencies.lock <https://github.com/pennaerial/monorepo/blob/main/payload_controller/dependencies.lock>`_
     - :doc:`components`
     - Managed Eigen dependency and resolved versions
   * - `eim_config.toml <https://github.com/pennaerial/monorepo/blob/main/payload_controller/eim_config.toml>`_
     - :doc:`getting-started`
     - ESP-IDF installation and shell tool paths
   * - `main/CMakeLists.txt <https://github.com/pennaerial/monorepo/blob/main/payload_controller/main/CMakeLists.txt>`_
     - :doc:`build-system`
     - Source/backend selection and component dependencies
   * - `main/payload_controller.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/main/payload_controller.cpp>`_
     - :doc:`overview`
     - Linux versus ESP startup and update loop
   * - `scripts/clang-tidy.sh <https://github.com/pennaerial/monorepo/blob/main/payload_controller/scripts/clang-tidy.sh>`_
     - :doc:`development-tools`
     - Static-analysis configuration and command/filter
   * - `scripts/sim.sh <https://github.com/pennaerial/monorepo/blob/main/payload_controller/scripts/sim.sh>`_
     - :doc:`simulation`
     - Simulation startup, transport threads or message conversion
   * - `sdkconfig.defaults <https://github.com/pennaerial/monorepo/blob/main/payload_controller/sdkconfig.defaults>`_
     - :doc:`build-system`
     - Build commands, target configuration, Gazebo linkage
   * - `sdkconfig.defaults.esp32s3 <https://github.com/pennaerial/monorepo/blob/main/payload_controller/sdkconfig.defaults.esp32s3>`_
     - :doc:`build-system`
     - Build commands, target configuration, Gazebo linkage
   * - `sdkconfig.defaults.linux <https://github.com/pennaerial/monorepo/blob/main/payload_controller/sdkconfig.defaults.linux>`_
     - :doc:`build-system`
     - Build commands, target configuration, Gazebo linkage
