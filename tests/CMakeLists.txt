cmake_minimum_required(VERSION 3.24)

# 1. Define the list of test source files
set(TEST_SOURCES
    motion/test_checked_model_factories.cpp
    math/test_checked_primitive_factories.cpp
    math/test_checked_covariance_factories.cpp
    math/test_functions.cpp
    math/test_matrix.cpp
    math/test_vector.cpp
    math/test_point2d.cpp
    math/test_rotation2d.cpp
    math/test_point3d.cpp
    math/test_matrix_column_view.cpp
    math/test_matrix_row_view.cpp
    math/test_matrix_view.cpp
    math/test_square_matrix.cpp
    math/test_square_matrix_decompositions.cpp
    math/test_triangular_matrix.cpp
    math/test_diagonal_matrix.cpp
    math/test_rank1_update.cpp
    math/test_modified_gram_schmidt.cpp
    math/test_covariance_matrix_full.cpp
    math/test_covariance_matrix_factored.cpp
    math/test_conversions.cpp
    math/test_matrix_io.cpp
    env/test_ego_motion.cpp
    motion/test_state_mem.cpp
    motion/test_motion_model_cv.cpp
    motion/test_motion_model_ca.cpp
    motion/test_measurement_update.cpp
    motion/test_measurement_update_nonlinear.cpp
    motion/test_information_filter_predict_psd.cpp
    motion/test_information_filter_predict_negative_q.cpp
    observation/test_position_observation_model.cpp
    observation/test_velocity_observation_model.cpp
    observation/test_range_bearing_observation_model.cpp
    observation/test_range_bearing_doppler_observation_model.cpp
    observation/test_bearing_observation_model.cpp
    observation/test_range_observation_model.cpp
    observation/test_sensor_mounting_pose.cpp
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

# Exercise public factory instantiations without the test-only access macros or
# GoogleTest. Keep exceptions-disabled flags local to these smoke executables.
foreach(smoke_target factory_smoke factory_smoke_ndebug)
    add_executable(${smoke_target} factory_smoke.cpp)
    target_include_directories(${smoke_target} PRIVATE ${PROJECT_SOURCE_DIR}/include)
    # expected v1.0.0 leaves its exception argument unused with -fno-exceptions.
    # Treat dependency headers as system headers on these targets only.
    target_include_directories(${smoke_target} SYSTEM PRIVATE $<TARGET_PROPERTY:expected,INTERFACE_INCLUDE_DIRECTORIES>)
    target_link_libraries(${smoke_target} PRIVATE trackingLib::trackingLib)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${smoke_target} PRIVATE -fno-exceptions)
    endif()
    add_test(NAME ${smoke_target} COMMAND ${smoke_target})
    list(APPEND ALL_TEST_EXECUTABLES ${smoke_target})
endforeach()
target_compile_definitions(factory_smoke_ndebug PRIVATE NDEBUG)

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
            # GCC-generated function end lines are classified as "mismatch"
            # by lcov 2.0 and "inconsistent" by newer versions.
            --ignore-errors unused,mismatch,inconsistent
            --filter brace
            --rc geninfo_unexecuted_blocks=0
            --rc derive_function_end_line=0
        GENHTML_ARGS
            --no-branch-coverage
            --ignore-errors mismatch,inconsistent
            --filter brace
            --legend
        EXCLUDE 
            "/Users/matthias/miniforge3/*" 
            "/Library/*" 
            "/usr/include/*" 
            "build_cov/*" 
            "_deps/*" 
            "examples/*" 
            "tests/*"
    )
endif()
