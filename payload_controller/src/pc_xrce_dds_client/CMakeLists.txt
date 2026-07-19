set(srcs
    pc_xrce_dds_client.cpp
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
)

foreach(_var XRCE_DDS_CLIENT_PATH XRCE_DDS_MICROCDR_PATH)
    if(NOT DEFINED ENV{${_var}})
        message(FATAL_ERROR
            "$ENV{${_var}} is not set. source Dependencies/monorepo-deps-env.sh "
            "before building payload_controller.")
    endif()
endforeach()

# Micro-CDR and Micro-XRCE-DDS-Client are built via ExternalProject_Add (each
# its own separate `cmake` configure+build), not add_subdirectory(). They
# were tried in-tree first, but ESP-IDF's ambient per-directory build state
# (its `linux` target's libc header shims + -pedantic -Werror, its implicit
# link against __idf_* component libs) kept leaking into them since
# add_subdirectory() shares the same CMake directory-scope tree. A separate
# process has none of that ambient state, so we forward only what these
# libraries actually need: the exact compiler/flags/toolchain ESP-IDF is
# already using for this target (so e.g. esp32s3's -mlongcalls stays
# consistent with the rest of the firmware image), nothing more.
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
# For other targets (e.g. esp32s3), don't forward CMAKE_C_FLAGS/CXX_FLAGS/
# EXE_LINKER_FLAGS at all: forwarding CMAKE_TOOLCHAIN_FILE above is enough on
# its own — that toolchain file unconditionally appends a GCC `@file`
# response-file reference (e.g. -mlongcalls, -specs=picolibc.specs) to
# whatever CMAKE_*_FLAGS already holds whenever it's reprocessed, pointing at
# the same shared, already-fully-populated build/<target>/toolchain/cflags
# path regardless of which build reprocesses it. Also forwarding our own
# copy of those same flags applies them twice on one command line, which GCC
# rejects outright for -specs ("attempt to rename spec... already defined").

ExternalProject_Add(microcdr_project
    PREFIX      ${xrce_deps_build_dir}/microcdr
    SOURCE_DIR  "$ENV{XRCE_DDS_MICROCDR_PATH}"
    CMAKE_CACHE_ARGS
        ${xrce_common_cache_args}
        -DUCDR_SUPERBUILD:BOOL=OFF
        -DUCDR_PIC:BOOL=OFF
        # Unlike the Client's equivalent (which defaults OFF), Micro-CDR
        # defaults this ON, which installs into a versioned subdirectory
        # (e.g. microcdr-2.0.1/lib/...) instead of flat lib/ — force it off
        # so both land at the flat path BUILD_BYPRODUCTS/IMPORTED_LOCATION
        # below expect.
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
    # Host build: ESP-IDF's `linux` toolchain doesn't override
    # CMAKE_SYSTEM_NAME, so the Client sees a real "Linux" system and its
    # native POSIX/termios serial backend just works.
    list(APPEND client_profile_args
        -DUCLIENT_PROFILE_SERIAL:BOOL=ON
        -DUCLIENT_PROFILE_CUSTOM_TRANSPORT:BOOL=OFF
    )
else()
    # esp32s3: ESP-IDF's toolchain sets CMAKE_SYSTEM_NAME to "Generic", which
    # the Client doesn't recognize as a POSIX-like platform, so its serial
    # profile wouldn't have a working backend. Use the custom-transport
    # profile instead, with UART glue supplied from pc_xrce_dds_client.cpp.
    list(APPEND client_profile_args
        -DUCLIENT_PROFILE_SERIAL:BOOL=OFF
        -DUCLIENT_PROFILE_CUSTOM_TRANSPORT:BOOL=ON
    )
endif()

ExternalProject_Add(xrce_dds_client_project
    PREFIX      ${xrce_deps_build_dir}/client
    SOURCE_DIR  "$ENV{XRCE_DDS_CLIENT_PATH}"
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
