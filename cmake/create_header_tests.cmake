# This CMake module provides a function to create empty main applications
# that include just one header from the trackinglib for all headers (h and hpp)
#
# Function: create_header_test_applications
# Creates executable targets that include individual headers to verify they are self-contained
#
# Usage: create_header_test_applications()
# This will create targets for all .h and .hpp files in the include/trackingLib directory

function(create_header_test_applications)
    # Find all header files in the trackinglib include directory
    file(GLOB_RECURSE TRACKINGLIB_HEADERS
        "${PROJECT_SOURCE_DIR}/include/trackingLib/*.h"
        "${PROJECT_SOURCE_DIR}/include/trackingLib/*.hpp"
    )
    
    # Create a directory for the header test applications
    set(HEADER_TEST_DIR "${PROJECT_BINARY_DIR}/header_tests")
    file(MAKE_DIRECTORY "${HEADER_TEST_DIR}")
    
    # Process each header file
    foreach(header_file ${TRACKINGLIB_HEADERS})
        # Get the relative path from include/trackingLib
        string(REPLACE "${PROJECT_SOURCE_DIR}/include/trackingLib/" "" relative_path "${header_file}")
        
        # Create a safe target name by replacing path separators and special characters
        string(REPLACE "/" "_" target_name "${relative_path}")
        string(REPLACE "." "_" target_name "${target_name}")
        string(REPLACE "-" "_" target_name "${target_name}")
        
        # Create the target name with prefix
        set(target_name "header_test_${target_name}")
        
        # Generate the main.cpp content for this header
        set(main_content 
"#include <trackingLib/${relative_path}>

int main() {
    // Empty main to test header inclusion
    return 0;
}"
        )
        
        # Write the main.cpp file
        set(main_file "${HEADER_TEST_DIR}/${target_name}.cpp")
        file(WRITE "${main_file}" "${main_content}")
        
        # Create the executable target
        add_executable(${target_name} "${main_file}")
        
        # Set include directories
        target_include_directories(${target_name}
            PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        )
        
        # Link with trackingLib
        target_link_libraries(${target_name} PUBLIC trackingLib::trackingLib)
        
        # Register test with CTest
        add_test(NAME ${target_name} COMMAND ${target_name})
        
        # Add to a custom target for easy building
        if(NOT TARGET header_tests)
            add_custom_target(header_tests)
        endif()
        add_dependencies(header_tests ${target_name})
        
        # Print status
        message(STATUS "Created header test target: ${target_name} for ${relative_path}")
    endforeach()
    
    message(STATUS "Created ${target_name} header test applications")
endfunction()