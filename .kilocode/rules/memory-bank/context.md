# Context

## Current State

The trackinglib project is a mature, well-structured C++ header-only library for object tracking. Recent major refactoring efforts have significantly improved code quality, eliminated circular dependencies, and established consistent design patterns across the codebase.

## Recent Activity (2026-01-01 to 2026-01-02)

### Completed Major Refactorings

1. **Print Methods Refactoring** (COMPLETED)
   - Replaced all `print()` methods with idiomatic C++ `operator<<`
   - Created centralized [`matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h) with template-based implementation
   - Eliminated code duplication across all matrix types
   - Removed circular dependencies caused by print methods
   - All 215 tests passing

2. **Cyclic Dependencies Resolution** (COMPLETED)
   - Created centralized conversion system in [`math/linalg/conversions/`](include/trackingLib/math/linalg/conversions/)
   - Established unified `<target>From<source>` naming convention
   - Implemented function overloading for list-based conversions
   - Separated decomposition implementations into [`square_matrix_decompositions.hpp`](include/trackingLib/math/linalg/square_matrix_decompositions.hpp)
   - Eliminated all circular dependencies between matrix classes
   - All 206 tests passing

3. **Motion Layer Initialization Strategy** (COMPLETED)
   - Added factory methods to [`ExtendedMotionModel`](include/trackingLib/motion/imotion_model.h) base class
   - Implemented `StateVecFromList()`, `StateCovFromList()`, and `FromLists()` methods
   - All motion models automatically inherit convenient initialization
   - No code duplication across motion models
   - All 206 tests passing

4. **Matrix Base Refactoring** (PARTIALLY COMPLETED)
   - Fixed aliasing detection in `operator+=` and `operator-=`
   - Made error handling consistent for division operators
   - Fixed `const` correctness in non-member operators
   - Added comprehensive test coverage for `setBlock()`, aliasing, transpose, and matrix multiplication
   - Standardized use of `Rows`/`Cols` vs `Rows_`/`Cols_` template parameters
   - Added `[[nodiscard]]` attributes to prevent misuse
   - Remaining: Documentation improvements

### Test Coverage Improvements
- Created comprehensive tests for [`MatrixView`](tests/math/test_matrix_view.cpp)
- Created comprehensive tests for [`Point2d`](tests/math/test_point2d.cpp) and [`Point3d`](tests/math/test_point3d.cpp)
- Added extensive tests for matrix operations (setBlock, aliasing, transpose, multiplication)
- Test count increased from 194 to 215 tests

## Project Status

### Implemented Features
- Extended Kalman Filter (EKF) with full and factored covariance support
- Information Filter (IF) with full and factored covariance support
- Constant Velocity (CV) motion model with ego motion compensation
- Constant Acceleration (CA) motion model with ego motion compensation
- Self-contained matrix library with comprehensive operations
- UDU factored covariance matrices for numerical stability
- Comprehensive unit test suite with high coverage
- AUTOSAR C++14 compliant codebase

### Experimental/Draft Features
- Unscented Kalman Filter (UKF) - stub only, not implemented
- Python bindings via pybind11 - experimental
- C++20 contracts - experimental

### Known Limitations
- Measurement update not yet implemented in all filters
- ego motion compensations is currently deactivated during prediction and also not tested
- UKF is not yet implemented (only header stub exists)
- Python bindings are experimental and may not be fully functional
- Eigen is only used for development/comparison, not in production code

## Next Steps

The project is in a stable state with comprehensive documentation. Future work could focus on:
- Increasing test coverage on math and other layers
- Fix all gcc and clang compiler warnings
- Refactor the contracts and have a special built step to check them on C++20 build
- Implement and activate the ego motion compensation during prediction
- Implementing measurement update for all filters
- Implementing the UKF filter
- Stabilizing Python bindings
- Adding more motion models if needed
- Expanding test coverage for edge cases
- Adding more examples and tutorials

## Important Notes

- All core algorithms avoid dynamic memory allocation for deterministic behavior
- Error handling uses `tl::expected` pattern instead of exceptions
- Template-heavy design provides compile-time safety but may increase compile times
- Special test macros (`TEST_REMOVE_PRIVATE`, etc.) allow testing private members
- Testcode locally disables clang formatting to improve readability of matrix definitions
- The library follows strict AUTOSAR C++14 guidelines for automotive safety
