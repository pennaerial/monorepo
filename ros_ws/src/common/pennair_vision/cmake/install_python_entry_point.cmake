# Install a console launcher without invoking setuptools install_scripts.
# Usage: install_python_entry_point(executable_name python_module function_name)
# Install the importable package separately with ament_python_install_package().
function(install_python_entry_point executable_name module_name function_name)
    if(NOT ARGC EQUAL 3)
        message(FATAL_ERROR
            "install_python_entry_point requires an executable name, module, and function")
    endif()

    get_executable_path(python_interpreter Python3::Interpreter CONFIGURE)
    set(launcher "${CMAKE_CURRENT_BINARY_DIR}/${executable_name}")
    file(WRITE "${launcher}"
"#!${python_interpreter}
import sys
from ${module_name} import ${function_name}

if __name__ == '__main__':
    sys.exit(${function_name}())
")
    # Symlink installs need the generated file itself to be executable.
    file(CHMOD "${launcher}" PERMISSIONS
        OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE
    )

    install(PROGRAMS "${launcher}"
        DESTINATION "lib/${PROJECT_NAME}"
    )
endfunction()
