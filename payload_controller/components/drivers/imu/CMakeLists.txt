set(srcs "")
set(priv_requires DDS)
set(requires util)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND srcs "imu_sitl.cpp")
    list(APPEND priv_requires "sitl")
else()
    list(APPEND srcs "imu_hw.cpp")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    PRIV_REQUIRES ${priv_requires}
    REQUIRES ${requires}
)

if(IDF_TARGET STREQUAL "linux")
    include("${CMAKE_CURRENT_LIST_DIR}/../../gz_deps.cmake")
    link_gz_transport(${COMPONENT_LIB})
endif()
