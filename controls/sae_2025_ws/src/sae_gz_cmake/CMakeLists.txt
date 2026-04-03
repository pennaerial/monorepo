cmake_minimum_required(VERSION 3.8)
project(sae_gz_cmake)

find_package(ament_cmake REQUIRED)

install(DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package(CONFIG_EXTRAS "cmake/sae_gz_cmake-extras.cmake")
