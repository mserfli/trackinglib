cmake_minimum_required(VERSION 3.0.0)
project(trackingLib VERSION 0.1.0)

include(CTest)
enable_testing()

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=1)
add_compile_options(-fcolor-diagnostics)

add_executable(trackingLib main.cpp)
target_include_directories(trackingLib PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/../eigen-3.4.0")
target_compile_features(trackingLib PRIVATE cxx_std_14)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
