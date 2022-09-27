cmake_minimum_required(VERSION 3.12)

include(FetchContent)

FetchContent_Declare (
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.12.x
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE 
)

FetchContent_MakeAvailable( googletest )

set(TEST_BINARY "testRunner")

add_compile_definitions(TEST_BUILD)
add_executable(${TEST_BINARY})
target_sources(${TEST_BINARY} PRIVATE
    test.cpp
    test_point2d.cpp
    test_triangular_matrix.cpp
)
target_include_directories(
    ${TEST_BINARY}
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(${TEST_BINARY} PUBLIC trackingLib::trackingLib gtest gtest_main pthread)

target_compile_features(${TEST_BINARY} PRIVATE cxx_std_11)
