# Tracking Library

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)](https://en.wikipedia.org/wiki/C%2B%2B17)
[![CMake](https://img.shields.io/badge/CMake-3.24+-blue)](https://cmake.org/)
[![GCC](https://img.shields.io/badge/GCC-7+-red)](https://gcc.gnu.org/)
[![Clang](https://img.shields.io/badge/Clang-5+-green)](https://clang.llvm.org/)
[![clang--format](https://img.shields.io/badge/clang--format-18+-purple)](https://clang.llvm.org/docs/ClangFormat.html)
[![clang--tidy](https://img.shields.io/badge/clang--tidy-18+-orange)](https://clang.llvm.org/extra/clang-tidy/)
[![GoogleTest](https://img.shields.io/badge/GoogleTest-1.16.x-blue)](https://google.github.io/googletest/)
[![Doxygen](https://img.shields.io/badge/Doxygen-latest-blue)](https://www.doxygen.nl/)
[![lcov](https://img.shields.io/badge/lcov-latest-blue)](https://github.com/linux-test-project/lcov)

Academic C++ header-only library for object tracking using Kalman filter variants.

## Architecture

For an overview of the core classes and their relationships, see [Architecture Diagram](doc/info_architecture.md).

## Prediction Flow

The sequence flow for filter predictions, including differences between Kalman and Information filters, is documented in [Prediction Flow](doc/info_filter_prediction.md).

## Key Concepts

**Library Design**: Header-only C++ library for academic object tracking using Extended Kalman Filter (EKF) and Information Filter (IF) variants on the available motion models. All motion models can be configured to use a factored or normal covariance matrix and have a predictor with built-in support for the ego motion compensation. 

The factored implementations are mainly based on publications from D'Souza, Bierman, Thornton, Carlson.
* C. D'Souza and R. Zanetti, "Information Formulation of the UDU Kalman Filter," in IEEE Transactions on Aerospace and Electronic Systems, vol. 55, no. 1, pp. 493-498, Feb. 2019, doi: 10.1109/TAES.2018.2850379.
* Pourtakdoust, Seid H. "Ud Covariance Factorization For Unscented Kalman Filter Using Sequential Measurements Update," 2007, doi:10.5281/ZENODO.1071229.
* Gerald J. Bierman, "Factorization Methods for Discrete Sequential Estimation", 1977
* Catherine L. Thornton, "Triangular Covariance Factorizations for Kalman Filtering", 1976
* Philip E. Gill, "Practical optimization", 2019, doi.org/10.1137/1.9781611975604


**Core Components**:
- **Motion Models**: Constant Velocity (CV) and Constant Acceleration (CA) with ego motion compensation
- **Matrix Library**: Self-contained linear algebra library with UDU factored covariance matrices for enhanced numerical stability, ensuring positive semidefiniteness by design
- **Filter Variants**: EKF and IF with both full and factored covariance support

**Key Requirements**:
- **Standards**: C++17 minimum, AUTOSAR C++14 compliant
- **Safety**: No exceptions (tl::expected pattern), no dynamic allocation in core algorithms
- **Quality**: Zero compile warnings, Comprehensive unit tests, >90% line coverage target, comprehensive Doxygen documentation

**Design Patterns**:
- **Template-Based**: Compile-time type safety and dimension checking
- **CRTP Interfaces**: Curiously Recurring Template Pattern for static polymorphism
- **Policy-Based Design**: Covariance matrix policies (Full/Factored) for flexible implementations
- **Centralized Conversions**: `<target>From<source>` naming for type conversions
- **Factory Methods**: Convenient initialization via `FromLists()` methods

**Modern C++ Features**:
- **C++20 Concepts**: Used for strict API contracts without inheritance, enabling compile-time interface checking
- **Contract Classes**: Custom contract system with `RequireCopyIntf`, `RequireMoveIntf` for compile-time enforcement of class capabilities
- **Template Metaprogramming**: Extensive use of templates for type safety and zero-runtime-overhead abstractions

**Tools & Technologies**:
- **Build**: CMake 3.24+, GCC/Clang compilers
- **Testing**: GoogleTest framework, lcov coverage
- **Quality**: clang-format, clang-tidy, Doxygen
- **Error Handling**: tl::expected (Rust-style Result pattern)


## Planned Functionalities

### Measurement Update Implementation (In Progress)

The library is undergoing a comprehensive update to implement measurement updates.
- **Observation Model Framework**: New C++17 framework with pure abstract `IObservationModel`, `ExtendedObservationModel`, and concrete models (Position, Range-Bearing, Velocity, Range-Bearing-Doppler).
- **GenericUpdate Implementation**: Support for sequential, block, and composed update modes.
- **Filter-Specific Updates**: Optimized measurement updates for both Kalman and Information filters.
- **Motion Model Integration**: Extended motion models with update methods supporting multiple observation models.

## Building

### Prerequisites

- CMake 3.24 or higher
- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- For documentation: Doxygen and Graphviz
- For coverage reports (Linux): lcov

### Quick Start

```bash
# Create build directory
mkdir build && cd build

# Configure with CMake (default: Debug build, C++17)
cmake ..

# Build the library and examples
cmake --build .

# Run tests
ctest --output-on-failure

# Build examples (optional, enabled by default)
cmake --build . --target single_object_tracking
./single_object_tracking
```

### Build Options

- **Build Type**: Set with `-DCMAKE_BUILD_TYPE=Release` for optimized builds
- **C++ Standard**: Set with `-DCMAKE_CXX_STANDARD=20` for C++20 features
- **Examples**: Disable with `-DBUILD_EXAMPLES=OFF`
- **Tests**: Enable with `-DBUILD_TESTING=ON`
- **Header Tests**: Enable with `-DBUILD_HEADER_TESTS=ON`

### Documentation

```bash
# Generate Doxygen documentation
doxygen
# Open html/index.html in your browser
```

### Coverage Report (Linux only)

```bash
# Generate coverage report
./report_coverage.sh
# Open build_cov/coverage/index.html in your browser
```

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.
