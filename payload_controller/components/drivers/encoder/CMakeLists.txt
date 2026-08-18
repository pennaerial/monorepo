set(srcs "")

if(IDF_TARGET STREQUAL "linux")
    list(APPEND srcs "encoder_sitl.cpp")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
)

if(IDF_TARGET STREQUAL "linux")
    include("${CMAKE_SOURCE_DIR}/cmake/gz_deps.cmake")
    link_gz_transport(${COMPONENT_LIB})
endif()
