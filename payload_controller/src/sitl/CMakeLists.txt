set(srcs sitl_runtime.cpp)

if(IDF_TARGET STREQUAL "linux")
    idf_component_register(
        SRCS ${srcs}
        INCLUDE_DIRS ".")
endif()
