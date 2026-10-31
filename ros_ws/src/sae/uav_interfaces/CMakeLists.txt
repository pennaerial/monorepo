cmake_minimum_required(VERSION 3.5)
project(uav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# set all .msg files
set(MSG_FILES
    msg/UdpTest.msg
    msg/HelloWorld.msg
)

# set all .srv files
set(SRV_FILES
    srv/CameraData.srv
    srv/PayloadAprilTagState.srv
    srv/PayloadColorOrbitState.srv
    srv/PayloadColorSquareState.srv
    srv/PayloadDriveOutState.srv
    srv/PayloadTracking.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces sensor_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
