cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)  # Produces gnu++20

if(IDF_TARGET STREQUAL "linux")
    set(COMPONENTS main)
endif()

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/drivers"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(payload_controller)


