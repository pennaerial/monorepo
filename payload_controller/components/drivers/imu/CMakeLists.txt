set(srcs "")
set(priv_requires DDS)
set(requires util)

set(sitl_srcs
    imu_sitl.cpp
    imu.cpp
)

set(hardware_srcs
    imu_hw.cpp
)


if(IDF_TARGET STREQUAL linux)
    list(APPEND srcs ${sitl_srcs})
    list(APPEND priv_requires sitl)
else()
    list(APPEND srcs ${hardware_srcs})
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    PRIV_REQUIRES ${priv_requires}
    REQUIRES ${requires}
)

if(IDF_TARGET STREQUAL "linux")
    include("${CMAKE_SOURCE_DIR}/cmake/gz_deps.cmake")
    link_gz_transport(${COMPONENT_LIB})
endif()
