# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

if(IDF_TARGET STREQUAL "linux")
    set(COMPONENTS main)
endif()

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/drivers"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(payload_controller)



