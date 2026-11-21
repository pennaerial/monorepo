set(srcs camera.cpp)
set(priv_requires DDS)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND srcs camera_sitl.cpp)
    list(APPEND priv_requires sitl)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES util
    PRIV_REQUIRES ${priv_requires}
)

if(IDF_TARGET STREQUAL "linux")
    include("${CMAKE_SOURCE_DIR}/cmake/gz_deps.cmake")
    link_gz_transport(${COMPONENT_LIB})
endif()
