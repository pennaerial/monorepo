cmake_minimum_required(VERSION 3.8)
project(pennair_cli)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(pennair_cli)

install(
  PROGRAMS
    script/pennair
  DESTINATION lib/${PROJECT_NAME} # run with `ros2 run pennair_cli pennair`
)


ament_package()
