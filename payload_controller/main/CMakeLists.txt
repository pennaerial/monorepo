
SET (priv_requires imu encoder DDS payload)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND priv_requires "sitl")
endif()

idf_component_register(SRCS "payload_controller.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES ${priv_requires})

# payload.hpp includes the SITL driver headers on linux, which include gz headers.
if(IDF_TARGET STREQUAL "linux")
include("${CMAKE_SOURCE_DIR}/cmake/gz_deps.cmake")
link_gz_transport(${COMPONENT_LIB})
endif()