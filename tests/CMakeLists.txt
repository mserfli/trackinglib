cmake_minimum_required(VERSION 3.24)

set(TEST_BINARY "testRunner")

if(UNIX AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
    setup_target_for_coverage_lcov(
        NAME lcov
        EXECUTABLE ${TEST_BINARY}
        BASE_DIRECTORY "${PROJECT_SOURCE_DIR}"
        EXCLUDE "/Users/matthias/miniforge3/*" "/Library/*" "/usr/include/*" "build/*" "_deps/*" "examples/*" "python/*" "tests/*"
    )
endif()

add_compile_definitions(TEST_BUILD)
add_executable(${TEST_BINARY})
target_sources(${TEST_BINARY} PRIVATE
    test.cpp
    math/test_matrix.cpp
    math/test_vector.cpp
    math/test_point2d.cpp
    math/test_matrix_column_view.cpp
    math/test_matrix_row_view.cpp
    math/test_square_matrix.cpp
    math/test_triangular_matrix.cpp
    math/test_diagonal_matrix.cpp
    math/test_rank1_update.cpp
    math/test_covariance_matrix_full.cpp
    math/test_covariance_matrix_factored.cpp
    motion/test_state_mem.cpp
    motion/test_motion_model_cv.cpp
    motion/test_motion_model_ca.cpp
)
target_include_directories(
    ${TEST_BINARY}
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(${TEST_BINARY} PUBLIC trackingLib::trackingLib gmock gmock_main pthread)

gtest_discover_tests(${TEST_BINARY})