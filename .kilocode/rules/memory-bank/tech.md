# Technology Stack

## Core Technologies

### Programming Language
- **C++17**: Minimum required standard
- **C++20**: Optional support for contracts (experimental)
- **Compilers**: GCC and Clang (both supported)

### Build System
- **CMake**: Version 3.24+ required
- **Project Version**: 0.3.0
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
- **Documentation Style** (see [`matrix.h`](include/trackingLib/math/linalg/matrix.h) for reference):
  - Use triple-slash comments (`///`) for all Doxygen documentation
  - Use backslash commands: `\brief`, `\tparam`, `\param[in]`, `\param[out]`, `\return`
  - Do NOT use @ commands (e.g., `@brief`, `@param`)
  - Include `\brief` for all classes and functions
  - Include `\tparam` for all template parameters
  - Include `\param[in]` or `\param[out]` for all function parameters
  - Include `\return` for all return values
  - Include `\note` for important implementation details
  - Include `\warning` for usage warnings
  - Include `\see` for cross-references
  - Use LaTeX notation for mathematical expressions where appropriate

### Testing

#### GoogleTest Framework
- **Test Organization**:
  - Main test runner: [`tests/test.cpp`](tests/test.cpp)
  - Math tests: [`tests/math/`](tests/math/)
    - [`test_matrix.cpp`](tests/math/test_matrix.cpp) - Matrix class tests (108 tests)
    - [`test_matrix_io.cpp`](tests/math/test_matrix_io.cpp) - Stream output tests (11 tests)
    - [`test_matrix_view.cpp`](tests/math/test_matrix_view.cpp) - MatrixView tests (4 tests)
    - [`test_matrix_row_view.cpp`](tests/math/test_matrix_row_view.cpp) - MatrixRowView tests (3 tests)
    - [`test_matrix_column_view.cpp`](tests/math/test_matrix_column_view.cpp) - MatrixColumnView tests (5 tests)
    - [`test_vector.cpp`](tests/math/test_vector.cpp) - Vector tests (8 tests)
    - [`test_point2d.cpp`](tests/math/test_point2d.cpp) - Point2d tests (4 tests)
    - [`test_point3d.cpp`](tests/math/test_point3d.cpp) - Point3d tests (4 tests)
    - [`test_square_matrix.cpp`](tests/math/test_square_matrix.cpp) - SquareMatrix tests (12 tests)
    - [`test_triangular_matrix.cpp`](tests/math/test_triangular_matrix.cpp) - TriangularMatrix tests (26 tests)
    - [`test_diagonal_matrix.cpp`](tests/math/test_diagonal_matrix.cpp) - DiagonalMatrix tests (18 tests)
    - [`test_covariance_matrix_full.cpp`](tests/math/test_covariance_matrix_full.cpp) - Covariance tests (6 tests)
    - [`test_covariance_matrix_factored.cpp`](tests/math/test_covariance_matrix_factored.cpp) - Factored covariance tests (8 tests)
    - [`test_rank1_update.cpp`](tests/math/test_rank1_update.cpp) - Rank-1 update tests (6 tests)
    - [`test_conversions.cpp`](tests/math/test_conversions.cpp) - Conversion functions tests (30-40 tests)
    - [`test_modified_gram_schmidt.cpp`](tests/math/test_modified_gram_schmidt.cpp) - Modified Gram-Schmidt tests (6 tests)
  - Motion tests: [`tests/motion/`](tests/motion/)
    - [`test_motion_model_cv.cpp`](tests/motion/test_motion_model_cv.cpp) - CV model tests
    - [`test_motion_model_ca.cpp`](tests/motion/test_motion_model_ca.cpp) - CA model tests
    - [`test_state_mem.cpp`](tests/motion/test_state_mem.cpp) - State memory tests
  - Mock objects: [`tests/motion/mocks/`](tests/motion/mocks/)
  
- **Test Patterns** (see [`test_matrix.cpp`](tests/math/test_matrix.cpp) for reference):
  - **Naming Convention**: `<operation>__<expected_result>`
    - Examples: `ctor_Zeros__Success`, `op_at__FailBadRowIdx`, `FromList_TooFewRows__ThrowsRuntimeError`
    - Use `// NOLINT` to suppress clang-tidy warnings on test function names
  - **Typed Tests** (`TYPED_TEST`): For testing template classes with different template parameters
    - Example: Testing both row-major and column-major matrix layouts
    - Create helper struct to wrap template parameters (e.g., `MatrixStorageType<bool IsRowMajor_>`)
    - Use `TYPED_TEST_SUITE(TestClass, TypeList)` to define type list
    - Example from [`test_matrix.cpp`](tests/math/test_matrix.cpp):
      ```cpp
      template <bool IsRowMajor_>
      struct MatrixStorageType { static constexpr auto IsRowMajor = IsRowMajor_; };
      
      using Implementations = Types<MatrixStorageType<true>, MatrixStorageType<false>>;
      TYPED_TEST_SUITE(GTestMatrix, Implementations);
      
      TYPED_TEST(GTestMatrix, ctor_Zeros__Success) { /* test code */ }
      ```
  - **Parameterized Tests** (`TEST_P`): For testing same functionality with different input values
    - Example: Testing decompositions with different matrix sizes or properties
    - Use `INSTANTIATE_TEST_SUITE_P` to provide parameter values
  - **Mock Objects**: For testing protected/private members
  - **clang-format Control**: Disable locally for matrix definitions to improve readability
    - Use `// clang-format off` and `// clang-format on` around matrix initializer lists
  - **Comprehensive Coverage**: Edge cases (aliasing, transpose, block operations)
  - **Test Count**: 215 tests (as of 2026-01-05)
  - **Test Coverage Plan**: Comprehensive plan created in [`plans/math_layer_test_coverage_plan.md`](plans/math_layer_test_coverage_plan.md)
  - **Target**: 310+ tests for math layer with >90% line coverage and >85% branch coverage
 
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
