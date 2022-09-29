cmake_minimum_required(VERSION 3.12)

set(PY_MODULE_NAME "pytracking")

# allow access to private and protected members
add_compile_definitions(TEST_BUILD)

# add python binding
pybind11_add_module(${PY_MODULE_NAME} main.cpp)

# MODULE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(${PY_MODULE_NAME} PRIVATE VERSION_INFO=${MODULE_VERSION_INFO})

target_compile_features(${PY_MODULE_NAME} PRIVATE cxx_std_20)

target_include_directories(
  ${PY_MODULE_NAME}
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(${PY_MODULE_NAME} PUBLIC trackingLib::trackingLib)
