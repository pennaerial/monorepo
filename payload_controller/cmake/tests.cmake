include(CMakeParseArguments)

function(register_linux_test TEST_NAME)
    cmake_parse_arguments(
        TEST
        ""
        ""
        "LINK_LIBRARIES"
        ${ARGN}
    )

    add_executable(
        ${TEST_NAME}
        ${TEST_UNPARSED_ARGUMENTS}
    )

    target_link_libraries(
        ${TEST_NAME}
        PRIVATE
            ${COMPONENT_LIB}
            unity
            ${TEST_LINK_LIBRARIES}
    )

    add_test(
        NAME ${TEST_NAME}
        COMMAND ${TEST_NAME}
    )
endfunction()
