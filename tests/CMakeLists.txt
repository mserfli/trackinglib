cmake_minimum_required(VERSION 3.12)

set(TEST_BINARY "testRunner")

setup_target_for_coverage_lcov(
          NAME lcov
          EXECUTABLE ${TEST_BINARY}
          BASE_DIRECTORY "${PROJECT_SOURCE_DIR}"
          EXCLUDE "/Users/matthias/miniforge3/*" "/Library/*" "/usr/include/*" "build/*" "_deps/*" "examples/*" "python/*" "tests/*")

add_compile_definitions(TEST_BUILD)
add_executable(${TEST_BINARY})
target_sources(${TEST_BINARY} PRIVATE
    test.cpp
    test_point2d.cpp
    test_triangular_matrix.cpp
    test_diagonal_matrix.cpp
    test_covariance_matrix_factored.cpp
    test_state_mem.cpp
    test_motion_model_cv.cpp
)
target_include_directories(
    ${TEST_BINARY}
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(${TEST_BINARY} PUBLIC trackingLib::trackingLib gtest gtest_main pthread)

target_compile_features(${TEST_BINARY} PRIVATE cxx_std_11)

gtest_discover_tests(${TEST_BINARY})