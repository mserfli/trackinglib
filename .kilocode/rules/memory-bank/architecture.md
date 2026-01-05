# Architecture

## System Overview

trackinglib is a header-only C++ library with a layered architecture designed for object tracking using Kalman filter variants. The library follows a template-based design pattern for compile-time type safety and flexibility.

## Directory Structure

```
include/trackingLib/
├── base/                    # Foundation layer
│   ├── atomic_types.h       # Type aliases (sint32, float32, etc.)
│   ├── first_include.h      # Must be first include in all files
│   ├── interface_contract.h # Contract definitions
│   └── reject_*/require_*   # Interface contracts for rule of 5
├── math/                    # Mathematics layer
│   ├── analysis/
│   │   └── functions.h      # Mathematical functions
│   └── linalg/              # Linear algebra
│       ├── matrix.h/.hpp    # General matrix implementation
│       ├── square_matrix.h/.hpp
│       ├── square_matrix_decompositions.hpp  # Decomposition implementations
│       ├── triangular_matrix.h/.hpp
│       ├── diagonal_matrix.h/.hpp
│       ├── vector.h/.hpp
│       ├── covariance_matrix_full.h/.hpp
│       ├── covariance_matrix_factored.h/.hpp  # UDU factorization
│       ├── rank1_update.h/.hpp
│       ├── modified_gram_schmidt.h/.hpp
│       ├── point2d.h, point3d.h
│       ├── matrix_io.h      # Stream output operators (operator<<)
│       ├── matrix_view.h/.hpp  # Non-owning matrix views
│       ├── matrix_row_view.h/.hpp
│       ├── matrix_column_view.h/.hpp
│       ├── contracts/       # Interface definitions
│       └── conversions/     # Centralized conversion system
│           ├── conversions.h              # Master header with forward declarations
│           ├── matrix_conversions.hpp     # Matrix conversions
│           ├── diagonal_conversions.hpp   # DiagonalMatrix conversions
│           ├── square_conversions.hpp     # SquareMatrix conversions
│           ├── triangular_conversions.hpp # TriangularMatrix conversions
│           ├── vector_conversions.hpp     # Vector conversions
│           └── covariance_matrix_conversions.hpp  # Covariance conversions
├── filter/                  # Filter implementations
│   ├── kalman_filter.h/.hpp           # Extended Kalman Filter
│   ├── unscented_kalman_filter.h      # UKF (stub)
│   ├── information_filter.h/.hpp      # Information Filter
│   └── covariance_intersection.h      # Covariance intersection
└── motion/                  # Motion models
    ├── imotion_model.h      # Motion model interface with factory methods
    ├── motion_model_cv.h/.hpp  # Constant Velocity
    ├── motion_model_ca.h/.hpp  # Constant Acceleration
    ├── generic_predict.h/.hpp  # Generic prediction logic
    ├── state_mem.h/.hpp     # State memory management
    └── contracts/           # Motion model contracts
```

## Key Components

### 1. Base Layer (`include/trackingLib/base/`)

**Purpose**: Provides fundamental types and contracts

**Key Files**:
- [`atomic_types.h`](include/trackingLib/base/atomic_types.h): Type aliases for portability
  - `sint32`, `uint32`, `float32`, `float64`
- [`first_include.h`](include/trackingLib/base/first_include.h): Must be included first in all headers
- [`interface_contract.h`](include/trackingLib/base/interface_contract.h): C++20 contract support
- Interface contracts: `reject_copy_intf.h`, `require_move_intf.h`, etc.

### 2. Math Layer (`include/trackingLib/math/linalg/`)

**Purpose**: Self-contained linear algebra library

**Key Classes**:
- [`Matrix<ValueType, Rows, Cols, IsRowMajor>`](include/trackingLib/math/linalg/matrix.h): General matrix with configurable storage
  - Supports both row-major and column-major layouts
  - Template-based with compile-time dimension checking
  - Uses `tl::expected` for error handling
  - Operations: arithmetic, transpose, block operations, Frobenius norm
  
- [`SquareMatrix<ValueType, Size, IsRowMajor>`](include/trackingLib/math/linalg/square_matrix.h): Square matrix specialization
  - Identity matrix support
  - Determinant, trace operations
  
- [`TriangularMatrix<ValueType, Size, IsLower, IsRowMajor>`](include/trackingLib/math/linalg/triangular_matrix.h): Upper/lower triangular
  
- [`DiagonalMatrix<ValueType, Size>`](include/trackingLib/math/linalg/diagonal_matrix.h): Diagonal matrix
  
- [`Vector<ValueType, Size>`](include/trackingLib/math/linalg/vector.h): Column vector
  
- [`CovarianceMatrixFull<FloatType, Size>`](include/trackingLib/math/linalg/covariance_matrix_full.h): Standard covariance
  
- [`CovarianceMatrixFactored<FloatType, Size>`](include/trackingLib/math/linalg/covariance_matrix_factored.h): UDU factored covariance
  - Stores U (unit upper triangular) and D (diagonal)
  - Thornton update for prediction
  - Agee-Turner rank-1 update
  - Better numerical stability

**Design Patterns**:
- Header/implementation split (`.h`/`.hpp`)
- Template-based for type flexibility
- CRTP (Curiously Recurring Template Pattern) for interfaces
- Error handling via `tl::expected<T, Errors>`
- Centralized conversion system with `<target>From<source>` naming pattern
- Function overloading for list-based conversions
- Stream output via template `operator<<` (no `print()` methods)

**Matrix I/O System** ([`matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h)):
- Template-based `operator<<` for all matrix types
- SFINAE-based type detection for C++17 compatibility
- Works with any `std::ostream` (cout, files, stringstream)
- Accesses elements via `at_unsafe()` interface (future-proof for packed storage)
- Specialized formatting for `DiagonalMatrix`
- No code duplication across matrix types

**Conversion System** ([`math/linalg/conversions/`](include/trackingLib/math/linalg/conversions/)):
- Centralized location for all type conversions
- Unified `<target>From<source>` naming convention
- Function overloading for different parameter types
- Eliminates circular dependencies between matrix classes
- Key conversions:
  - `DiagonalFromSquare()`, `DiagonalFromList()` (overloaded)
  - `SquareFromDiagonal()`, `SquareFromList()`
  - `TriangularFromSquare()`, `TriangularFromList()`
  - `VectorFromMatrixColumnView()`, `VectorFromList()`
  - `MatrixFromVector()`, `MatrixFromMatrixColumnView()`
  - `CovarianceMatrixFromList()` (supports both full and factored)

### 3. Filter Layer (`include/trackingLib/filter/`)

**Purpose**: Kalman filter implementations

**Key Classes**:
- [`KalmanFilter<FloatType>`](include/trackingLib/filter/kalman_filter.h): Extended Kalman Filter
  - Static methods for prediction
  - Supports both full and factored covariance
  - Template method: `predictCovariance<DimX, DimQ>(P, A, G, Q)`
  
- [`InformationFilter<FloatType>`](include/trackingLib/filter/information_filter.h): Information form
  - Works with information matrix (inverse covariance)
  - Supports both full and factored forms
  
- [`UnscentedKalmanFilter`](include/trackingLib/filter/unscented_kalman_filter.h): UKF (stub, not implemented)

**Filter Interface**:
```cpp
template <sint32 DimX_, sint32 DimQ_>
static void predictCovariance(
    CovarianceMatrix& P,
    const SquareMatrix<FloatType_, DimX_>& A,
    const Matrix<FloatType_, DimX_, DimQ_>& G,
    const DiagonalMatrix<FloatType_, DimQ_>& Q
);
```

### 4. Motion Model Layer (`include/trackingLib/motion/`)

**Purpose**: Motion models with ego motion compensation

**Key Classes**:
- [`IMotionModel<FloatType>`](include/trackingLib/motion/imotion_model.h): Base interface
  - Virtual methods: `getX()`, `getVx()`, `getAx()`, `getY()`, `getVy()`, `getAy()`
  - Virtual `predict()` methods for different filters
  
- [`ExtendedMotionModel<MotionModel, CovarianceMatrixType, FloatType, Size>`](include/trackingLib/motion/imotion_model.h): CRTP base
  - Combines `IMotionModel` and `StateMem`
  - Provides common position accessors
  - Factory methods for convenient initialization:
    - `StateVecFromList()` - Create state vector from initializer list
    - `StateCovFromList()` - Create covariance from initializer list
    - `FromLists()` - Create complete motion model from lists
  
- [`MotionModelCV<CovarianceMatrixType, FloatType>`](include/trackingLib/motion/motion_model_cv.h): Constant Velocity
  - State: `[X, VX, Y, VY]` (4D)
  - Process noise: `[Q_VX, Q_VY]` (2D)
  - Methods: `predict()`, `compensateEgoMotion()`, `applyProcessModel()`, `computeA()`, `computeQ()`, `computeG()`
  - Supports conversion from CA model
  
- [`MotionModelCA<CovarianceMatrixType, FloatType>`](include/trackingLib/motion/motion_model_ca.h): Constant Acceleration
  - State: `[X, VX, AX, Y, VY, AY]` (6D)
  - Process noise: `[Q_AX, Q_AY]` (2D)
  - Similar interface to CV model
  - Supports conversion from CV model
  
- [`generic::Predict<MotionModel, FloatType, CovarianceMatrixType>`](include/trackingLib/motion/generic_predict.h): CRTP mixin
  - Provides generic prediction logic
  - Handles ego motion compensation
  - Works with both KalmanFilter and InformationFilter

**Motion Model Design**:
- Template parameter `CovarianceMatrixType` allows choosing between full and factored covariance
- Ego motion compensation built into prediction
- State transition: `x_k+1 = f(x_k, dt) + ego_motion_compensation`
- Covariance update: `P_k+1 = A*P*A' + G*Q*G' + ego_motion_terms`

### 5. State Management (`include/trackingLib/motion/`)

- [`StateMem<CovarianceMatrixType, FloatType, Size>`](include/trackingLib/motion/state_mem.h): State vector and covariance storage
- [`StateVecConverter`](include/trackingLib/motion/state_vec_converter.h): State vector conversions
- [`StateCovConverter`](include/trackingLib/motion/state_cov_converter.h): Covariance conversions

## Design Principles

### 1. Header-Only Library
- All implementation in headers (`.hpp` files)
- No compilation required for library itself
- Easy integration into projects

### 2. Template-Based Design
- Compile-time type checking
- Dimension checking at compile time
- Support for different floating-point types (`float32`, `float64`)
- Zero runtime overhead for abstractions

### 3. Error Handling
- Uses `tl::expected<T, Errors>` pattern (Rust-style)
- No exceptions in core library
- Explicit error propagation
- Error types defined in [`math/linalg/errors.h`](include/trackingLib/math/linalg/errors.h)

### 4. AUTOSAR C++14 Compliance
- Follows automotive safety standards
- Strict coding guidelines
- No dynamic memory allocation in core algorithms
- Deterministic behavior

### 5. Numerical Stability
- UDU factorization for covariance matrices
- Based on academic publications (Bierman, Thornton, D'Souza)
- Avoids matrix inversion where possible
- Rank-1 updates for efficiency

## OpenMP Decision

**Status**: Removed (2026-01-02)

**Rationale**:
- Typical matrix sizes (≤15×15) are too small to benefit from parallelization
- Thread creation overhead exceeds computation benefit for small matrices
- AUTOSAR C++14 compliance requires deterministic behavior
- OpenMP's non-deterministic thread scheduling conflicts with this requirement
- Simplifies codebase and removes external dependency

**Performance Impact**:
- No negative impact for typical use cases (4×4, 6×6, 15×15 matrices)
- Small matrices are faster without thread overhead
- Maintains consistent, predictable performance

**Historical Note**:
- Previous implementation had OpenMP in some matrix multiplications
- Implementation had syntax errors (undefined variables in private/shared clauses)
- Application was inconsistent across matrix types

## Critical Implementation Paths

### Prediction Flow (CV Model with EKF)
1. User calls [`MotionModelCV::predict(dt, filter, egoMotion)`](include/trackingLib/motion/motion_model_cv.h:91)
2. Delegates to [`generic::Predict::predict()`](include/trackingLib/motion/generic_predict.hpp)
3. Compensates ego motion: [`compensateEgoMotion(Ge, Go, egoMotion)`](include/trackingLib/motion/motion_model_cv.h:107)
4. Applies process model: [`applyProcessModel(dt)`](include/trackingLib/motion/motion_model_cv.h:111)
5. Computes matrices: [`computeA(A, dt)`](include/trackingLib/motion/motion_model_cv.h:116), [`computeG(G, dt)`](include/trackingLib/motion/motion_model_cv.h:126), [`computeQ(Q, dt)`](include/trackingLib/motion/motion_model_cv.h:121)
6. Updates covariance: [`KalmanFilter::predictCovariance(P, A, G, Q)`](include/trackingLib/filter/kalman_filter.h:22)

### UDU Factored Covariance Update
1. [`CovarianceMatrixFactored::thornton(Phi, G, Q)`](include/trackingLib/math/linalg/covariance_matrix_factored.h:101)
2. Implements Thornton's algorithm for `P = Phi*P*Phi' + G*Q*G'`
3. Maintains UDU factorization throughout
4. Numerically stable for ill-conditioned matrices

## Testing Strategy

- Comprehensive unit tests in [`tests/`](tests/)
- GoogleTest framework
- Typed tests for template instantiations
- Coverage measured with lcov
- Test organization mirrors source structure:
  - [`tests/math/`](tests/math/): Math library tests
  - [`tests/motion/`](tests/motion/): Motion model tests
  - Mock objects in [`tests/motion/mocks/`](tests/motion/mocks/)
- Comprehensive test coverage plan in [`plans/math_layer_test_coverage_plan.md`](plans/math_layer_test_coverage_plan.md)
- Target: 310+ tests with >90% line coverage and >85% branch coverage

## Build System

- CMake-based build system ([`CMakeLists.txt`](CMakeLists.txt:1))
- Version: 0.3.0
- Minimum C++ standard: C++17
- Optional dependencies:
  - Eigen (development only, via `USE_EIGEN` option)
  - pybind11 (Python bindings, via `USE_PYBIND11` option)
- Required dependencies:
  - GoogleTest (fetched automatically)
  - tl::expected (fetched automatically)

## Code Quality Tools

- **clang-format**: Code formatting ([`.clang-format`](.clang-format:1))
  - Based on Microsoft style
  - 130 column limit
  - Pointer alignment left
  
- **clang-tidy**: Static analysis ([`.clang-tidy`](.clang-tidy:1))
  - Enabled checks: bugprone, google, misc, modernize, performance, portability, readability
  - Warnings as errors
  
- **Doxygen**: Documentation generation ([`Doxyfile`](Doxyfile:1))
  - Generates HTML documentation
  - Call graphs enabled
  - UML-style diagrams

## Special Macros

- `TEST_REMOVE_FINAL`: Removes `final` keyword in test builds
- `TEST_REMOVE_PROTECTED`: Changes `protected` to `public` in test builds
- `TEST_REMOVE_PRIVATE`: Changes `private` to `public` in test builds
- `TEST_VIRTUAL`: Adds `virtual` keyword in test builds for mocking

These macros allow testing of private/protected members while maintaining proper encapsulation in production builds.
