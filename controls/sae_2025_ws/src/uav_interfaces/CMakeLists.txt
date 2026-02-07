cmake_minimum_required(VERSION 3.5)
project(uav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find all .msg and .srv files
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/msg/*.msg")
file(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
