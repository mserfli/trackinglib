cmake_minimum_required(VERSION 3.0.0)

add_executable(matrix)
target_sources(matrix PRIVATE
    main.cpp
)
target_include_directories(
    matrix
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(matrix PUBLIC trackingLib::trackingLib)
