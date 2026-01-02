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

4. **Matrix Base Refactoring** (COMPLETED)
     - Fixed aliasing detection in `operator+=` and `operator-=`
     - Made error handling consistent for division operators
     - Fixed `const` correctness in non-member operators
     - Added comprehensive test coverage for `setBlock()`, aliasing, transpose, and matrix multiplication
     - Standardized use of `Rows`/`Cols` vs `Rows_`/`Cols_` template parameters
     - Added `[[nodiscard]]` attributes to prevent misuse
     - Completed comprehensive documentation improvements for Matrix class

5. **OpenMP Removal** (COMPLETED)
    - Completely removed all OpenMP pragmas from matrix multiplication operations
    - Removed 6 pragmas total: 4 from `triangular_matrix.hpp`, 2 from `matrix_view.hpp`
    - Eliminated syntax errors and inconsistent parallelization
    - Maintains AUTOSAR C++14 deterministic behavior requirements
    - All 206 tests passing

6. **Math Layer Documentation Phase 1.1** (COMPLETED 2026-01-02)
    - Enhanced comprehensive class-level documentation for [`Matrix`](include/trackingLib/math/linalg/matrix.h)
    - Added detailed template parameter constraints and storage layout notes
    - Documented tl::expected error handling patterns throughout
    - Added usage examples for common Matrix operations
    - Improved documentation for all public methods with complete \brief, \tparam, \param[in], \return coverage
    - Doxygen generates without warnings for Matrix class

7. **Math Layer Documentation Phase 1.2** (COMPLETED 2026-01-02)
    - Added comprehensive class-level documentation for [`SquareMatrix`](include/trackingLib/math/linalg/square_matrix.h)
    - Added comprehensive class-level documentation for [`TriangularMatrix`](include/trackingLib/math/linalg/triangular_matrix.h)
    - Added comprehensive class-level documentation for [`DiagonalMatrix`](include/trackingLib/math/linalg/diagonal_matrix.h)
    - Enhanced method documentation with mathematical background, complexity analysis, and usage notes
    - Documented decomposition algorithms (QR, LLT, LDLT, UDUT) with references to academic literature
    - Added performance characteristics and numerical stability notes
    - Doxygen generates without warnings for all three classes

8. **Math Layer Documentation Phase 1.3** (COMPLETED 2026-01-02)
    - Added comprehensive class-level documentation for [`Vector`](include/trackingLib/math/linalg/vector.h)
    - Documented all public methods with \brief, \tparam, \param[in], \return
    - Added mathematical notation for vector operations (dot product, norm, normalize)
    - Documented relationship to Matrix class
    - Doxygen generates without warnings for Vector class
    - All 206 tests passing (no regressions)

9. **Math Layer Documentation Phase 2.1** (COMPLETED 2026-01-02)
    - Added comprehensive file-level documentation for [`matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h)
    - Documented template-based operator<< design with SFINAE-based type detection
    - Documented is_matrix_like trait and helper variable
    - Added detailed examples for general matrices and DiagonalMatrix specialization
    - Documented formatting behavior for different matrix types and value types
    - Doxygen generates without warnings for matrix_io.h
    - All 206 tests passing (no regressions)

10. **Math Layer Documentation Phase 2.2** (COMPLETED 2026-01-02)
    - Added comprehensive file-level documentation for [`conversions/conversions.h`](include/trackingLib/math/linalg/conversions/conversions.h)
    - Documented centralized conversion architecture and design principles
    - Documented `<target>From<source>` naming convention
    - Added cross-references to all conversion implementation files
    - Added comprehensive Doxygen documentation for all conversion functions in each .hpp file
    - Documented function overloading strategy and error handling
    - Each function has \brief, \tparam, \param[in], \return, and \see tags
    - Doxygen generates without warnings for conversion system
    - All 206 tests passing (no regressions)
    - **Architecture Decision**: Conversion function declarations remain in implementation files to avoid circular dependencies
    - **Documentation Strategy**: Each conversion function will be documented in its respective `.hpp` implementation file
    - **Rationale**: Maintains the circular-dependency-free design while providing comprehensive documentation

11. **Math Layer Documentation Phase 2.3** (COMPLETED 2026-01-02)
    - Added comprehensive Doxygen documentation for all four decomposition algorithms in [`square_matrix.h`](include/trackingLib/math/linalg/square_matrix.h) declarations
    - Corrected documentation location: moved from implementation files to header declarations to avoid duplication
    - Documented Householder QR decomposition with mathematical background and numerical stability notes
    - Documented Cholesky (LLT) decomposition with preconditions and error handling
    - Documented LDL^T decomposition with numerical advantages over LLT
    - Documented UDU^T decomposition with Kalman filtering context and academic references
    - Added complexity analysis (O(n^3)) and space complexity notes for all decompositions
    - Included academic references (Grewal & Andrews, Bierman, Thornton, etc.)
    - Removed redundant documentation from implementation files in [`square_matrix_decompositions.hpp`](include/trackingLib/math/linalg/square_matrix_decompositions.hpp)
    - Doxygen generates without warnings for decomposition functions
    - All 206 tests passing (no regressions)

12. **Math Layer Documentation Phase 3.1** (COMPLETED 2026-01-02)
    - Added comprehensive class-level documentation for [`MatrixRowView`](include/trackingLib/math/linalg/matrix_row_view.h)
    - Added comprehensive class-level documentation for [`MatrixColumnView`](include/trackingLib/math/linalg/matrix_column_view.h)
    - Documented non-owning view concept for single row/column views
    - Documented lifetime and safety considerations
    - Documented all supported operations (matrix multiplication, dot products, element access)
    - Added usage examples and performance notes (zero-copy operations)
    - Documented all public methods with \brief, \tparam, \param[in], \return
    - Added cross-references between view classes (MatrixView, MatrixRowView, MatrixColumnView)
    - Doxygen generates without warnings for both view classes
    - All 206 tests passing (no regressions)

13. **Math Layer Documentation Phase 4.1** (COMPLETED 2026-01-02)
    - Added comprehensive class-level documentation for [`CovarianceMatrixFull`](include/trackingLib/math/linalg/covariance_matrix_full.h)
    - Added comprehensive class-level documentation for [`CovarianceMatrixFactored`](include/trackingLib/math/linalg/covariance_matrix_factored.h)
    - Documented covariance matrix properties (symmetry, positive semi-definiteness)
    - Documented UDU factorization concept and numerical stability benefits
    - Documented Thornton update algorithm for Kalman filter prediction
    - Documented Agee-Turner rank-1 update for measurement updates
    - Added references to academic publications (Thornton, Bierman, D'Souza)
    - Enhanced method documentation with complexity analysis and usage notes
    - Doxygen generates without warnings for both covariance matrix classes
    - All 206 tests passing (no regressions)

14. **Math Layer Documentation Phase 5.1** (COMPLETED 2026-01-02)
    - Added comprehensive class-level documentation for [`Rank1Update`](include/trackingLib/math/linalg/rank1_update.h)
    - Added comprehensive class-level documentation for [`ModifiedGramSchmidt`](include/trackingLib/math/linalg/modified_gram_schmidt.h)
    - Documented rank-1 update algorithms for UDU and LDL factorizations
    - Documented Modified Gram-Schmidt orthogonalization for UDU factorization
    - Added mathematical background and numerical stability considerations
    - Referenced academic sources (Gill et al., Thornton, Grewal & Andrews)
    - Documented Kalman filtering applications and usage contexts
    - Doxygen generates without warnings for both specialized operation classes
    - All 206 tests passing (no regressions)

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
