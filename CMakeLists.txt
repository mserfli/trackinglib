cmake_minimum_required(VERSION 3.24)

project(trackingLib 
        VERSION 0.1.0
)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include(FetchContent)

FetchContent_Declare (
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.12.x
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE 
)

FetchContent_Declare (
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE 
)

FetchContent_Declare (
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG stable
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE 
)

FetchContent_Declare (
    expected
    GIT_REPOSITORY https://github.com/TartanLlama/expected.git
    GIT_TAG v1.0.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE 
)

option(EIGEN_BUILD_DOC OFF)
option(BUILD_TESTING OFF)
option(EIGEN_BUILD_PKGCONFIG OFF)
option(EXPECTED_ENABLE_TESTS OFF)
option(PYBIND11_TEST OFF)

FetchContent_MakeAvailable (
    googletest
    Eigen
    pybind11
    expected
)

# activate code coverage
include(coverage)
append_coverage_compiler_flags()

enable_testing()
include(GoogleTest)

# generate compile_commands.json used by clang-tidy
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=1)

# create header only library 
# based on https://github.com/bernedom/SI and https://discourse.cmake.org/t/header-only-library/5476
add_library(${PROJECT_NAME} INTERFACE)
# add alias so the project can be uses with add_subdirectory
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

include(GNUInstallDirs)

# Adding the install interface generator expression makes sure that the include
# files are installed to the proper location (provided by GNUInstallDirs)
target_include_directories(
    ${PROJECT_NAME}
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_14)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen expected)

# add_subdirectory(examples)
# add_subdirectory(python)
add_subdirectory(tests)

# locations are provided by GNUInstallDirs
install(
    TARGETS ${PROJECT_NAME}
    EXPORT  ${PROJECT_NAME}_Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(
    EXPORT ${PROJECT_NAME}_Targets
    FILE ${PROJECT_NAME}Targets.cmake 
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(
    FILES 
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION 
        ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/trackingLib
    DESTINATION include
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
