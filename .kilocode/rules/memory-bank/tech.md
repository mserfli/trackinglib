# Technology Stack

## Core Technologies

### Programming Language
- **C++17**: Minimum required standard
- **C++20**: Optional support for contracts (experimental)
- **Compilers**: GCC and Clang (both supported)

### Build System
- **CMake**: Version 3.24+ required
- **Project Version**: 0.2.0
- **Build Types**: Debug, Release, Coverage

### Dependencies

#### Required (Automatically Fetched)
- **GoogleTest**: v1.16.x
  - Unit testing framework
  - Fetched via CMake FetchContent
  
- **tl::expected**: v1.0.0
  - Rust-style error handling
  - Repository: https://github.com/TartanLlama/expected
  - Provides `tl::expected<T, E>` for safe error propagation

#### Optional
- **Eigen**: 3.4
  - Linear algebra library
  - Used for development/comparison only
  - Enabled via `USE_EIGEN` CMake option
  
- **pybind11**: stable branch
  - Python bindings (experimental)
  - Enabled via `USE_PYBIND11` CMake option

## Development Tools

### Code Quality

#### clang-format
- **Configuration**: [`.clang-format`](.clang-format)
- **Base Style**: Microsoft
- **Column Limit**: 130
- **Pointer Alignment**: Left
- **Key Settings**:
  - No bin packing for arguments/parameters
  - Consecutive alignment for assignments and declarations
  - Template declarations always break

#### clang-tidy
- **Configuration**: [`.clang-tidy`](.clang-tidy)
- **Enabled Checks**:
  - bugprone-*
  - google-*
  - misc-*
  - modernize-*
  - performance-*
  - portability-*
  - readability-*
- **Warnings as Errors**: Enabled
- **Disabled Checks**:
  - modernize-concat-nested-namespaces
  - modernize-make-unique
  - readability-avoid-const-params-in-decls
  - readability-identifier-length
  - bugprone-easily-swappable-parameters
  - google-readability-namespace-comments
  - google-runtime-int
  - google-runtime-references
  - misc-non-private-member-variables-in-classes
  - readability-named-parameter
  - readability-braces-around-statements
  - readability-magic-numbers

#### Doxygen
- **Configuration**: [`Doxyfile`](Doxyfile)
- **Output**: HTML documentation in `./doxydoc/`
- **Features**:
  - Call graphs enabled
  - UML-style class diagrams
  - Graphviz integration (dot)
  - Interactive SVG output
  - Extract all entities (including private)
- **Special Macros**:
  - `TEST_REMOVE_PROTECTED=protected`
  - `TEST_REMOVE_PRIVATE=private`
  - `TEST_VIRTUAL=`

### Testing

#### GoogleTest Framework
- **Test Organization**:
  - Main test runner: [`tests/test.cpp`](tests/test.cpp)
  - Math tests: [`tests/math/`](tests/math/)
  - Motion tests: [`tests/motion/`](tests/motion/)
  - Mock objects: [`tests/motion/mocks/`](tests/motion/mocks/)
  
- **Test Patterns**:
  - Typed tests for template instantiations
  - Parameterized tests for different configurations
  - Mock objects for testing protected/private members
  - disable clang format locally on matrix definitions to improve readability 

#### Code Coverage
- **Tool**: lcov
- **Build Type**: Coverage
- **Commands**:
  ```bash
  cmake -DCMAKE_BUILD_TYPE=Coverage ..
  cmake --build . --target lcov
  ```
- **Configuration**: [`cmake/coverage.cmake`](cmake/coverage.cmake)

## Compiler Flags

### Unix (GCC/Clang)
- **Common**: `-Wall -Wextra`
- **Debug**: `-g`
- **Release**: `-O3`
- **Coverage**: Appended via coverage.cmake

### Windows (MSVC)
- **Common**: `/W0 /EHsc`
- **Debug**: `/DEBUG`
- **Release**: `/O2`

## Standards Compliance

### AUTOSAR C++14
- **Guidelines**: https://sbmueller.github.io/autosar_cpp_guidelines/index.html
- **Key Requirements**:
  - No dynamic memory allocation in core algorithms
  - Deterministic behavior
  - Strict coding standards
  - Output parameters not used (RVO/NRVO instead)

### C++ Standard Features Used
- **Templates**: Extensive use for type safety and flexibility
- **CRTP**: Curiously Recurring Template Pattern for static polymorphism
- **constexpr**: Compile-time constants
- **std::array**: Fixed-size containers
- **std::expected**: Error handling (via tl::expected)
- **Type traits**: SFINAE and enable_if for template specialization

## Python Integration (Experimental)

### Setup
- **Build System**: setuptools with CMake extension
- **Configuration**: [`setup.py`](setup.py)
- **Package Name**: pytracking
- **Version**: 0.0.1
- **Installation**: `pip install .`

### Requirements
- Python 3.6+
- pybind11 (fetched automatically)
- CMake build system integration

## Docker Support

### Dockerfiles
- **Linux**: [`Dockerfile`](Dockerfile)
- **macOS**: [`DockerfileMac`](DockerfileMac)
- **Development Environment**: [`dev-env.sh`](dev-env.sh)

## Project Structure

### Header-Only Library
- **Include Path**: `include/trackingLib/`
- **Implementation Files**: `.hpp` files alongside `.h` headers
- **Installation**: Headers copied to system include directory

### CMake Integration
- **Config File**: [`cmake/trackingLibConfig.cmake.in`](cmake/trackingLibConfig.cmake.in)
- **Export**: Targets exported for use with `find_package()`
- **Namespace**: `trackingLib::`

## Development Workflow

### Build Commands
```bash
# Standard build
mkdir build && cd build
cmake ..
cmake --build .

# With Eigen support
cmake -DUSE_EIGEN=ON ..

# With Python bindings
cmake -DUSE_PYBIND11=ON ..

# Coverage build
cmake -DCMAKE_BUILD_TYPE=Coverage ..
cmake --build . --target lcov
```

### Documentation Generation
```bash
# Generate Doxygen documentation
doxygen
# Output in ./doxydoc/
```

### Testing
```bash
# Run all tests
cd build
ctest

# Or run test executable directly
./tests/test
```

## Technical Constraints

1. **No Exceptions**: Core library uses `tl::expected` for error handling
2. **No Dynamic Allocation**: All memory allocated at compile time via templates
3. **Header-Only**: No separate compilation step required
4. **Template-Heavy**: Compile times may be longer, but runtime is optimal
5. **C++17 Minimum**: Cannot use C++20 features except in experimental code
6. **AUTOSAR Compliant**: Must follow strict automotive coding guidelines
