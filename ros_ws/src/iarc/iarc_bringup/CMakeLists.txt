cmake_minimum_required(VERSION 3.8)
project(iarc_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY missions
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
