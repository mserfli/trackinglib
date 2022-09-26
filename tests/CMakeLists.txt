cmake_minimum_required(VERSION 3.12)

include(FetchContent)

FetchContent_Declare (
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.12.x
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE 
)

FetchContent_MakeAvailable (
    googletest
)

add_executable(test)
target_sources(test PRIVATE
    test.cpp
    test_point2d.cpp
)
target_include_directories(
  test
  PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(test PUBLIC trackingLib::trackingLib gtest gtest_main pthread)

target_compile_features(test PRIVATE cxx_std_11)
