cmake_minimum_required(VERSION 3.24)

# 1. Define the list of test source files
set(TEST_SOURCES
    math/test_matrix.cpp
    math/test_vector.cpp
    math/test_point2d.cpp
    math/test_point3d.cpp
    math/test_matrix_column_view.cpp
    math/test_matrix_row_view.cpp
    math/test_matrix_view.cpp
    math/test_square_matrix.cpp
    math/test_triangular_matrix.cpp
    math/test_diagonal_matrix.cpp
    math/test_rank1_update.cpp
    math/test_covariance_matrix_full.cpp
    math/test_covariance_matrix_factored.cpp
    math/test_conversions.cpp
    math/test_modified_gram_schmidt.cpp
    motion/test_state_mem.cpp
    motion/test_motion_model_cv.cpp
    motion/test_motion_model_ca.cpp
)

set(ALL_TEST_EXECUTABLES "")

# 2. Loop through each file to create a standalone executable
foreach(test_source ${TEST_SOURCES})
    # Extract a name for the executable (e.g., test_matrix)
    get_filename_component(test_name ${test_source} NAME_WE)
    
    add_executable(${test_name} test.cpp ${test_source})
    
    target_compile_definitions(${test_name} PRIVATE TEST_BUILD)
    
    target_include_directories(${test_name}
        PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    )
    
    target_link_libraries(${test_name} 
        PRIVATE 
            trackingLib::trackingLib 
            gmock 
            gmock_main 
            pthread
    )
    
    # Enable test discovery for this specific executable
    gtest_discover_tests(${test_name})
    
    # Collect the target name for coverage dependencies
    list(APPEND ALL_TEST_EXECUTABLES ${test_name})
endforeach()

# 3. Setup unified coverage
# By using 'ctest' as the EXECUTABLE, lcov will collect data from all 
# the individual test runs executed by the CTest runner.
if(UNIX AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
    setup_target_for_coverage_lcov(
        NAME lcov
        EXECUTABLE ctest # CTest runs all individual executables
        EXECUTABLE_ARGS "--output-on-failure"
        DEPENDENCIES ${ALL_TEST_EXECUTABLES}
        BASE_DIRECTORY "${PROJECT_SOURCE_DIR}"
        LCOV_ARGS 
            --ignore-errors unused,mismatch 
            --rc lcov_excl_braces_stop=1 
            --rc geninfo_unexecuted_blocks=0
            --rc derive_function_end_line=0
        GENHTML_ARGS
            --no-branch-coverage
            --ignore-errors mismatch
        EXCLUDE 
            "/Users/matthias/miniforge3/*" 
            "/Library/*" 
            "/usr/include/*" 
            "build/*" 
            "_deps/*" 
            "examples/*" 
            "python/*" 
            "tests/*"
    )
endif()
