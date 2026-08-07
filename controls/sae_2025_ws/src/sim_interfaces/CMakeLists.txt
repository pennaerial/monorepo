cmake_minimum_required(VERSION 3.5)
project(sim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# set all .msg files
set(MSG_FILES
    msg/HoopPose.msg
)

# set all .srv files
set(SRV_FILES
    srv/AttachDetach.srv
    srv/CreateMultiple.srv
    srv/HoopList.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces sensor_msgs ros_gz_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
