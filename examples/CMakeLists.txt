cmake_minimum_required(VERSION 3.24)

# Create executable for single linear object tracking example
add_executable(single_linear_object_tracking)
target_sources(single_linear_object_tracking PRIVATE
    single_linear_object_tracking.cpp
)
target_include_directories(
    single_linear_object_tracking
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(single_linear_object_tracking PUBLIC trackingLib::trackingLib)

# Create executable for single nonlinear object tracking example
add_executable(single_nonlinear_object_tracking)
target_sources(single_nonlinear_object_tracking PRIVATE
    single_nonlinear_object_tracking.cpp
)
target_include_directories(
    single_nonlinear_object_tracking
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(single_nonlinear_object_tracking PUBLIC trackingLib::trackingLib)

# Create executable for single nonlinear figure-8 object tracking example
add_executable(single_nonlinear_figure8_object_tracking)
target_sources(single_nonlinear_figure8_object_tracking PRIVATE
    single_nonlinear_figure8_object_tracking.cpp
)
target_include_directories(
    single_nonlinear_figure8_object_tracking
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(single_nonlinear_figure8_object_tracking PUBLIC trackingLib::trackingLib)
