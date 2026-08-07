cmake_minimum_required(VERSION 3.5)
project(payload_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# all .msg files
set(MSG_FILES
    msg/DriveCommand.msg
    msg/MotorState.msg
    msg/PayloadHeartbeat.msg
    msg/ServoCommand.msg
)

# all .srv files
set(SRV_FILES
    srv/ComputePidZieglerNichols.srv
    srv/DeadReckon.srv
    srv/TimedDrive.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
