cmake_minimum_required(VERSION 3.8)
project(pennair_cli)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(pennair_cli)

install(
  PROGRAMS
    script/pennair
  DESTINATION lib/${PROJECT_NAME} # install/pennair_cli/lib/pennair_cli/pennair
)


# this script puts the above install location into path
# after sourcing install/setup.bash
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hook/pennair_cli_path.sh.in"
)

ament_package()
