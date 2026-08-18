set(srcs sitl_runtime.cpp)

if(IDF_TARGET STREQUAL "linux")
    idf_component_register(
        SRCS ${srcs}
        INCLUDE_DIRS "."
        REQUIRES DDS
    )

    include("${CMAKE_CURRENT_LIST_DIR}/../gz_deps.cmake")
    link_gz_transport(${COMPONENT_LIB})
endif()
