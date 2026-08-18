set(srcs
    sitl_runtime.cpp
    convert.cpp
)

if(IDF_TARGET STREQUAL "linux")
    idf_component_register(
        SRCS ${srcs}
        INCLUDE_DIRS "."
        REQUIRES DDS
    )

    # from top-level CMakeLists.txt dir
    include("${CMAKE_SOURCE_DIR}/cmake/gz_deps.cmake")
    link_gz_transport(${COMPONENT_LIB})
endif()
