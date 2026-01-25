cmake_minimum_required(VERSION 3.24)

# Create executable for single object tracking example
add_executable(single_object_tracking)
target_sources(single_object_tracking PRIVATE
    single_object_tracking.cpp
)
target_include_directories(
    single_object_tracking
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(single_object_tracking PUBLIC trackingLib::trackingLib)