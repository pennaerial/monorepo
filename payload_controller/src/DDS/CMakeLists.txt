# DDS Client code

set(srcs
    dds_client.cpp
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
)

foreach(_var PENNAIR_XRCE_DDS_CLIENT_PATH PENNAIR_XRCE_DDS_MICROCDR_PATH)
    if(NOT DEFINED ENV{${_var}})
        message(FATAL_ERROR "$ENV{${_var}} is not set")
    else()
        message(STATUS "${_var} resolved to: $ENV{${_var}}")
    endif()
endforeach()

# Micro-CDR and Micro-XRCE-DDS-Client are built via ExternalProject_Add
include(ExternalProject)

set(xrce_deps_build_dir "${CMAKE_CURRENT_BINARY_DIR}/xrce_deps")
set(xrce_deps_install_dir "${xrce_deps_build_dir}/install")

set(xrce_common_cache_args
    -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
    -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_TOOLCHAIN_FILE:STRING=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${xrce_deps_install_dir}
    -DCMAKE_PREFIX_PATH:PATH=${xrce_deps_install_dir}
    -DBUILD_SHARED_LIBS:BOOL=OFF
)

if(IDF_TARGET STREQUAL "linux")
    # `linux`'s toolchain file is a near-empty placeholder — it doesn't set
    # up any response-file mechanism, so the real flags (fPIC, sdkconfig
    # defines, etc.) live directly in CMAKE_C_FLAGS. Forward them explicitly.
    list(APPEND xrce_common_cache_args
        -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
        -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
        -DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
    )
endif()

ExternalProject_Add(microcdr_project
    PREFIX      ${xrce_deps_build_dir}/microcdr
    SOURCE_DIR  "$ENV{PENNAIR_XRCE_DDS_MICROCDR_PATH}"
    CMAKE_CACHE_ARGS
        ${xrce_common_cache_args}
        -DUCDR_SUPERBUILD:BOOL=OFF
        -DUCDR_PIC:BOOL=OFF
        -DUCDR_ISOLATED_INSTALL:BOOL=OFF
    BUILD_BYPRODUCTS ${xrce_deps_install_dir}/lib/libmicrocdr.a
    LOG_CONFIGURE true
    LOG_BUILD true
    LOG_INSTALL true
    LOG_OUTPUT_ON_FAILURE true
)

# Profiles shared by both targets.
set(client_profile_args
    -DUCLIENT_PROFILE_UDP:BOOL=OFF
    -DUCLIENT_PROFILE_TCP:BOOL=OFF
    -DUCLIENT_PROFILE_DISCOVERY:BOOL=OFF
    -DUCLIENT_PROFILE_SHARED_MEMORY:BOOL=OFF
    -DUCLIENT_PROFILE_MULTITHREAD:BOOL=OFF
)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND client_profile_args
        -DUCLIENT_PROFILE_SERIAL:BOOL=ON
        -DUCLIENT_PROFILE_CUSTOM_TRANSPORT:BOOL=OFF
    )
else()
    # Use the custom-transport profile with UART config supplied from dds_client.cpp.
    list(APPEND client_profile_args
        -DUCLIENT_PROFILE_SERIAL:BOOL=OFF
        -DUCLIENT_PROFILE_CUSTOM_TRANSPORT:BOOL=ON
    )
endif()

ExternalProject_Add(xrce_dds_client_project
    PREFIX      ${xrce_deps_build_dir}/client
    SOURCE_DIR  "$ENV{PENNAIR_XRCE_DDS_CLIENT_PATH}"
    CMAKE_CACHE_ARGS
        ${xrce_common_cache_args}
        -DUCLIENT_SUPERBUILD:BOOL=OFF
        -DUCLIENT_PIC:BOOL=OFF
        ${client_profile_args}
    BUILD_BYPRODUCTS ${xrce_deps_install_dir}/lib/libmicroxrcedds_client.a
    DEPENDS microcdr_project
    LOG_CONFIGURE true
    LOG_BUILD true
    LOG_INSTALL true
    LOG_OUTPUT_ON_FAILURE true
)

add_library(microcdr_imported STATIC IMPORTED GLOBAL)
set_target_properties(microcdr_imported PROPERTIES
    IMPORTED_LOCATION ${xrce_deps_install_dir}/lib/libmicrocdr.a
)
add_dependencies(microcdr_imported microcdr_project)

add_library(microxrcedds_client_imported STATIC IMPORTED GLOBAL)
set_target_properties(microxrcedds_client_imported PROPERTIES
    IMPORTED_LOCATION ${xrce_deps_install_dir}/lib/libmicroxrcedds_client.a
)
add_dependencies(microxrcedds_client_imported xrce_dds_client_project)

target_include_directories(${COMPONENT_LIB} PRIVATE ${xrce_deps_install_dir}/include)
target_link_libraries(${COMPONENT_LIB} PRIVATE microxrcedds_client_imported microcdr_imported)


#################################################################################################
#                                      Msgs generation
#################################################################################################
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(IDL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/idl")

set(IDL_FILES
    "${IDL_PATH}/builtin_interfaces/msg/Time.idl"
    "${IDL_PATH}/geometry_msgs/msg/Quaternion.idl"
    "${IDL_PATH}/geometry_msgs/msg/Vector3.idl"
    "${IDL_PATH}/sensor_msgs/msg/Imu.idl"
    "${IDL_PATH}/std_msgs/msg/Header.idl"
)

file(MAKE_DIRECTORY "${GENERATED_DIR}")

message(STATUS "Python: ${Python3_EXECUTABLE}")
message(STATUS "Generator: ${CMAKE_CURRENT_SOURCE_DIR}/generate.py")
message(STATUS "IDL path: ${IDL_PATH}")
message(STATUS "Output: ${GENERATED_DIR}")
message(STATUS "IDL files: ${IDL_FILES}")

execute_process(
    COMMAND
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/generate.py"
        --idl-path "${IDL_PATH}"
        --o "${GENERATED_DIR}"
        --idl-files ${IDL_FILES}

    RESULT_VARIABLE GENERATE_RESULT
    OUTPUT_VARIABLE GENERATE_OUTPUT
    ERROR_VARIABLE GENERATE_ERROR
)

message(STATUS "Generator result: ${GENERATE_RESULT}")
message(STATUS "Generator stdout: ${GENERATE_OUTPUT}")
message(STATUS "Generator stderr: ${GENERATE_ERROR}")

if(NOT GENERATE_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to generate Micro XRCE-DDS interfaces")
endif()

set(GENERATED_SOURCES
    "${GENERATED_DIR}/builtin_interfaces/msg/Time.c"
    "${GENERATED_DIR}/geometry_msgs/msg/Quaternion.c"
    "${GENERATED_DIR}/geometry_msgs/msg/Vector3.c"
    "${GENERATED_DIR}/sensor_msgs/msg/Imu.c"
    "${GENERATED_DIR}/std_msgs/msg/Header.c"
)

target_sources(
    ${COMPONENT_LIB}
    PRIVATE
        ${GENERATED_SOURCES}
)

target_include_directories(
    ${COMPONENT_LIB}
    PUBLIC
        "${GENERATED_DIR}"
)
