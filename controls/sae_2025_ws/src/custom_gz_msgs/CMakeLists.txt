cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(custom_gz_msgs VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-msgs10 REQUIRED)

set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

set(MSGS_PROTOS
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/custom_msgs/dynamic_detachable_joint.proto
)

gz_msgs_generate_messages(
  TARGET msgs
  PROTO_PACKAGE "gz.custom_msgs"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  MSGS_PROTOS ${MSGS_PROTOS}
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

# Set include directory for consumers of the installed package
target_include_directories(custom_gz_msgs-msgs PUBLIC
  $<INSTALL_INTERFACE:include>
)

# Install the generated library
install(TARGETS custom_gz_msgs-msgs EXPORT custom_gz_msgs-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY ${CMAKE_BINARY_DIR}/custom_gz_msgs-msgs_genmsg/gz/custom_msgs/
  DESTINATION include/gz/custom_msgs
  FILES_MATCHING PATTERN "*.pb.h"
)

# Export CMake config so find_package(custom_gz_msgs) works
install(EXPORT custom_gz_msgs-targets
  FILE custom_gz_msgsConfig.cmake
  DESTINATION lib/cmake/custom_gz_msgs
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz_descriptor_path.sh.in"
)

ament_package()
