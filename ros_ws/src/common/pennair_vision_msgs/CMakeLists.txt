cmake_minimum_required(VERSION 3.5)
project(pennair_vision_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# all .msg files
set(MSG_FILES
    msg/Detection.msg
    msg/DetectionArray.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
