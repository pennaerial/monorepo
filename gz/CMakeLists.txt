cmake_minimum_required(VERSION 3.10)
project(vtol_motor_pub)
# find the packages you need
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
# create executable
add_executable(motor_pub main.cpp)
# link against the libraries
target_link_libraries(motor_pub
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
# include directories (optional, CMake targets handle this)
target_include_directories(motor_pub PRIVATE
  ${gz-transport13_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
)