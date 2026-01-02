# Math Layer Documentation and Test Coverage Analysis Plan

## Executive Summary

This plan addresses the need to analyze and improve Doxygen documentation and test coverage for the math layer of trackinglib. Recent refactoring efforts (print methods removal, cyclic dependency resolution, motion layer initialization) have resulted in new files being added and some unittest code being partially removed. This analysis will ensure comprehensive documentation and test coverage across all math layer components.

## Current State Analysis

### Math Layer Structure

The math layer consists of two main subdirectories:

#### 1. Analysis Layer (`include/trackingLib/math/analysis/`)
- [`functions.h`](include/trackingLib/math/analysis/functions.h) - Compile-time power function

#### 2. Linear Algebra Layer (`include/trackingLib/math/linalg/`)

**Core Matrix Classes:**
- [`matrix.h/.hpp`](include/trackingLib/math/linalg/matrix.h) - General matrix implementation
- [`square_matrix.h/.hpp`](include/trackingLib/math/linalg/square_matrix.h) - Square matrix specialization
- [`triangular_matrix.h/.hpp`](include/trackingLib/math/linalg/triangular_matrix.h) - Triangular matrices
- [`diagonal_matrix.h/.hpp`](include/trackingLib/math/linalg/diagonal_matrix.h) - Diagonal matrices
- [`vector.h/.hpp`](include/trackingLib/math/linalg/vector.h) - Column vectors

**View Classes:**
- [`matrix_view.h/.hpp`](include/trackingLib/math/linalg/matrix_view.h) - Non-owning matrix views
- [`matrix_row_view.h/.hpp`](include/trackingLib/math/linalg/matrix_row_view.h) - Row views
- [`matrix_column_view.h/.hpp`](include/trackingLib/math/linalg/matrix_column_view.h) - Column views

**Covariance Classes:**
- [`covariance_matrix_full.h/.hpp`](include/trackingLib/math/linalg/covariance_matrix_full.h) - Standard covariance
- [`covariance_matrix_factored.h/.hpp`](include/trackingLib/math/linalg/covariance_matrix_factored.h) - UDU factored covariance

**Specialized Operations:**
- [`square_matrix_decompositions.hpp`](include/trackingLib/math/linalg/square_matrix_decompositions.hpp) - LLT, LDLT, UDUT, QR decompositions
- [`rank1_update.h/.hpp`](include/trackingLib/math/linalg/rank1_update.h) - Rank-1 updates for factored matrices
- [`modified_gram_schmidt.h/.hpp`](include/trackingLib/math/linalg/modified_gram_schmidt.h) - MGS algorithm

**Utility Files:**
- [`matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h) - Stream output operators (recently added)
- [`matrix_types.h`](include/trackingLib/math/linalg/matrix_types.h) - Enumerations
- [`errors.h`](include/trackingLib/math/linalg/errors.h) - Error types
- [`block_diagonal_matrix.h`](include/trackingLib/math/linalg/block_diagonal_matrix.h) - Stub/TODO
- [`point2d.h`](include/trackingLib/math/linalg/point2d.h) - 2D point
- [`point3d.h`](include/trackingLib/math/linalg/point3d.h) - 3D point

**Contracts:**
- [`contracts/matrix_intf.h`](include/trackingLib/math/linalg/contracts/matrix_intf.h) - Matrix interface
- [`contracts/covariance_matrix_intf.h`](include/trackingLib/math/linalg/contracts/covariance_matrix_intf.h) - Covariance interface

**Conversions (Recently Added):**
- [`conversions/conversions.h`](include/trackingLib/math/linalg/conversions/conversions.h) - Master header
- [`conversions/matrix_conversions.hpp`](include/trackingLib/math/linalg/conversions/matrix_conversions.hpp)
- [`conversions/diagonal_conversions.hpp`](include/trackingLib/math/linalg/conversions/diagonal_conversions.hpp)
- [`conversions/square_conversions.hpp`](include/trackingLib/math/linalg/conversions/square_conversions.hpp)
- [`conversions/triangular_conversions.hpp`](include/trackingLib/math/linalg/conversions/triangular_conversions.hpp)
- [`conversions/vector_conversions.hpp`](include/trackingLib/math/linalg/conversions/vector_conversions.hpp)
- [`conversions/covariance_matrix_conversions.hpp`](include/trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp)

### Test Coverage Status

**Existing Test Files (14 files, 215 tests):**
- [`test_matrix.cpp`](tests/math/test_matrix.cpp) - 108 tests (comprehensive)
- [`test_square_matrix.cpp`](tests/math/test_square_matrix.cpp) - 12 tests
- [`test_triangular_matrix.cpp`](tests/math/test_triangular_matrix.cpp) - 26 tests
- [`test_diagonal_matrix.cpp`](tests/math/test_diagonal_matrix.cpp) - 18 tests
- [`test_vector.cpp`](tests/math/test_vector.cpp) - 8 tests
- [`test_matrix_view.cpp`](tests/math/test_matrix_view.cpp) - 4 tests (recently added)
- [`test_matrix_row_view.cpp`](tests/math/test_matrix_row_view.cpp) - 3 tests
- [`test_matrix_column_view.cpp`](tests/math/test_matrix_column_view.cpp) - 5 tests
- [`test_covariance_matrix_full.cpp`](tests/math/test_covariance_matrix_full.cpp) - 6 tests
- [`test_covariance_matrix_factored.cpp`](tests/math/test_covariance_matrix_factored.cpp) - 8 tests
- [`test_rank1_update.cpp`](tests/math/test_rank1_update.cpp) - 6 tests
- [`test_matrix_io.cpp`](tests/math/test_matrix_io.cpp) - 11 tests (recently added)
- [`test_point2d.cpp`](tests/math/test_point2d.cpp) - 4 tests (recently added)
- [`test_point3d.cpp`](tests/math/test_point3d.cpp) - 4 tests (recently added)

**Missing Test Files:**
- No tests for [`modified_gram_schmidt.h/.hpp`](include/trackingLib/math/linalg/modified_gram_schmidt.h)
- No tests for [`square_matrix_decompositions.hpp`](include/trackingLib/math/linalg/square_matrix_decompositions.hpp) (decompositions tested via square_matrix tests)
- No tests for conversion functions in [`conversions/`](include/trackingLib/math/linalg/conversions/) directory
- No tests for [`functions.h`](include/trackingLib/math/analysis/functions.h) (compile-time power function)
- No tests for [`matrix_types.h`](include/trackingLib/math/linalg/matrix_types.h) (enums)
- No tests for [`errors.h`](include/trackingLib/math/linalg/errors.h) (error types)
- No tests for contract interfaces

### Documentation Coverage Status

**Files with Minimal/No Doxygen Documentation:**

1. **Newly Added Files (from recent refactoring):**
   - [`matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h) - No Doxygen comments
   - All conversion files in [`conversions/`](include/trackingLib/math/linalg/conversions/) - No Doxygen comments
   - [`square_matrix_decompositions.hpp`](include/trackingLib/math/linalg/square_matrix_decompositions.hpp) - Minimal comments

2. **Utility Files:**
   - [`matrix_types.h`](include/trackingLib/math/linalg/matrix_types.h) - No Doxygen comments
   - [`errors.h`](include/trackingLib/math/linalg/errors.h) - No Doxygen comments
   - [`functions.h`](include/trackingLib/math/analysis/functions.h) - No Doxygen comments

3. **Point Classes:**
   - [`point2d.h`](include/trackingLib/math/linalg/point2d.h) - No Doxygen comments
   - [`point3d.h`](include/trackingLib/math/linalg/point3d.h) - No Doxygen comments

4. **Specialized Operations:**
   - [`modified_gram_schmidt.h/.hpp`](include/trackingLib/math/linalg/modified_gram_schmidt.h) - Minimal documentation
   - [`rank1_update.h/.hpp`](include/trackingLib/math/linalg/rank1_update.h) - Needs review

5. **View Classes:**
   - [`matrix_view.h/.hpp`](include/trackingLib/math/linalg/matrix_view.h) - Minimal documentation
   - [`matrix_row_view.h/.hpp`](include/trackingLib/math/linalg/matrix_row_view.h) - Minimal documentation
   - [`matrix_column_view.h/.hpp`](include/trackingLib/math/linalg/matrix_column_view.h) - Minimal documentation

6. **Covariance Classes:**
   - [`covariance_matrix_full.h/.hpp`](include/trackingLib/math/linalg/covariance_matrix_full.h) - Needs review
   - [`covariance_matrix_factored.h/.hpp`](include/trackingLib/math/linalg/covariance_matrix_factored.h) - Needs review

**Files with Partial Documentation:**
- [`matrix.h`](include/trackingLib/math/linalg/matrix.h) - Has some Doxygen comments but incomplete
- [`square_matrix.h`](include/trackingLib/math/linalg/square_matrix.h) - Needs review
- [`triangular_matrix.h`](include/trackingLib/math/linalg/triangular_matrix.h) - Needs review
- [`diagonal_matrix.h`](include/trackingLib/math/linalg/diagonal_matrix.h) - Needs review
- [`vector.h`](include/trackingLib/math/linalg/vector.h) - Needs review

## Documentation Improvement Plan

### Phase 1: High Priority - Core Classes (Estimated: 3-4 sessions)

#### 1.1 Matrix Base Class
**File:** [`matrix.h`](include/trackingLib/math/linalg/matrix.h)
- Add comprehensive class-level documentation
- Document all public methods with `\brief`, `\tparam`, `\param[in]`, `\return` (backslash style)
- Document template parameters and their constraints
- Add usage examples for common operations
- Document error handling patterns (tl::expected)
- Add notes about row-major vs column-major storage

#### 1.2 Specialized Matrix Classes
**Files:** [`square_matrix.h`](include/trackingLib/math/linalg/square_matrix.h), [`triangular_matrix.h`](include/trackingLib/math/linalg/triangular_matrix.h), [`diagonal_matrix.h`](include/trackingLib/math/linalg/diagonal_matrix.h)
- Document class purpose and relationship to base Matrix class
- Document specialized operations (determinant, trace, inverse, etc.)
- Add mathematical notation where appropriate
- Document performance characteristics
- Add usage examples

#### 1.3 Vector Class
**File:** [`vector.h`](include/trackingLib/math/linalg/vector.h)
- Document as column vector specialization
- Document vector operations (dot product, norm, normalize)
- Add mathematical notation
- Document relationship to Matrix class

### Phase 2: High Priority - Recently Added Files (Estimated: 2-3 sessions)

#### 2.1 Matrix I/O System
**File:** [`matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h)
- Document the template-based operator<< design
- Explain SFINAE-based type detection
- Document usage with different stream types
- Add examples for cout, file streams, stringstream
- Document formatting behavior for different matrix types

#### 2.2 Conversion System
**Files:** All files in [`conversions/`](include/trackingLib/math/linalg/conversions/) directory
- Document the centralized conversion architecture
- Explain `<target>From<source>` naming convention
- Document function overloading strategy
- Add usage examples for each conversion function
- Document error handling for invalid conversions
- Add cross-references between related conversions

#### 2.3 Decomposition Implementations
**File:** [`square_matrix_decompositions.hpp`](include/trackingLib/math/linalg/square_matrix_decompositions.hpp)
- Document each decomposition algorithm (Householder QR, LLT, LDLT, UDUT)
- Add mathematical background and references
- Document preconditions (symmetry, positive definiteness)
- Document numerical stability considerations
- Add complexity analysis
- Reference academic sources (Bierman, Thornton, etc.)

### Phase 3: Medium Priority - View Classes (Estimated: 2 sessions)

#### 3.1 Matrix Views
**Files:** [`matrix_view.h`](include/trackingLib/math/linalg/matrix_view.h), [`matrix_row_view.h`](include/trackingLib/math/linalg/matrix_row_view.h), [`matrix_column_view.h`](include/trackingLib/math/linalg/matrix_column_view.h)
- Document non-owning view concept
- Document lifetime and safety considerations
- Document supported operations
- Add usage examples
- Document performance characteristics (zero-copy)

### Phase 4: Medium Priority - Covariance Classes (Estimated: 2 sessions)

#### 4.1 Covariance Matrices
**Files:** [`covariance_matrix_full.h`](include/trackingLib/math/linalg/covariance_matrix_full.h), [`covariance_matrix_factored.h`](include/trackingLib/math/linalg/covariance_matrix_factored.h)
- Document covariance matrix properties (symmetry, positive semi-definiteness)
- Document UDU factorization concept and benefits
- Document Thornton update algorithm
- Document Agee-Turner rank-1 update
- Add references to academic publications
- Document numerical stability advantages
- Add usage examples for both forms

### Phase 5: Medium Priority - Specialized Operations (Estimated: 2 sessions)

#### 5.1 Rank-1 Updates
**File:** [`rank1_update.h`](include/trackingLib/math/linalg/rank1_update.h)
- Document rank-1 update concept
- Document UDU and LDL update/downdate algorithms
- Add mathematical notation
- Reference academic sources
- Document numerical considerations

#### 5.2 Modified Gram-Schmidt
**File:** [`modified_gram_schmidt.h`](include/trackingLib/math/linalg/modified_gram_schmidt.h)
- Document MGS algorithm purpose
- Document usage in UDU factorization context
- Add mathematical background
- Document numerical stability properties

### Phase 6: Low Priority - Utility Files (Estimated: 1 session)

#### 6.1 Type Definitions and Enums
**Files:** [`matrix_types.h`](include/trackingLib/math/linalg/matrix_types.h), [`errors.h`](include/trackingLib/math/linalg/errors.h)
- Document enum values and their meanings
- Document error types and when they occur
- Add usage examples

#### 6.2 Point Classes
**Files:** [`point2d.h`](include/trackingLib/math/linalg/point2d.h), [`point3d.h`](include/trackingLib/math/linalg/point3d.h)
- Document point classes as Vector specializations
- Document accessor methods
- Add usage examples

#### 6.3 Analysis Functions
**File:** [`functions.h`](include/trackingLib/math/analysis/functions.h)
- Document compile-time power function
- Add usage examples
- Document template metaprogramming approach

### Phase 7: Contract Interfaces (Estimated: 1 session)

#### 7.1 Interface Documentation
**Files:** [`contracts/matrix_intf.h`](include/trackingLib/math/linalg/contracts/matrix_intf.h), [`contracts/covariance_matrix_intf.h`](include/trackingLib/math/linalg/contracts/covariance_matrix_intf.h)
- Document interface purpose and design
- Document CRTP pattern usage
- Document contract requirements
- Add implementation guidelines

## Test Coverage Improvement Plan

### Phase 1: Critical Gaps - New Files (Estimated: 3-4 sessions)

#### 1.1 Conversion Functions Testing
**New File:** `tests/math/test_conversions.cpp`
- Test all conversion functions in [`conversions/`](include/trackingLib/math/linalg/conversions/) directory
- Use **Typed Tests** for conversions that work with different storage layouts (row-major/column-major)
- Test naming convention: `<ConversionName>__<expected_result>`
  - Examples: `DiagonalFromSquare__Success`, `DiagonalFromList_InvalidSize__ThrowsRuntimeError`
- Test `DiagonalFromSquare()`, `DiagonalFromList()`
- Test `SquareFromDiagonal()`, `SquareFromList()`
- Test `TriangularFromSquare()`, `TriangularFromList()`
- Test `VectorFromMatrixColumnView()`, `VectorFromList()`
- Test `MatrixFromVector()`, `MatrixFromMatrixColumnView()`
- Test `CovarianceMatrixFromList()` for both full and factored
- Test error cases (invalid dimensions, invalid data)
- Test function overloading behavior
- **Estimated:** 30-40 tests

#### 1.2 Modified Gram-Schmidt Testing
**New File:** `tests/math/test_modified_gram_schmidt.cpp`
- Test MGS algorithm for UDU factorization
- Use **Parameterized Tests** for different matrix sizes
- Test naming convention: `run_<transformation>__<expected_result>`
  - Examples: `run_PhiUDUPhiT__Success`, `run_PhiUDUPhiT_PlusGQGT__Success`
- Test `Phi*UDU'*Phi'` transformation: `run_PhiUDUPhiT__Success`
- Test `Phi*UDU'*Phi' + G*Q*G'` transformation: `run_PhiUDUPhiT_PlusGQGT__Success`
- Test with different matrix sizes (use parameterized tests)
- Test numerical stability: `run_NumericalStability__Success`
- Compare results with reference implementations
- **Estimated:** 10-15 tests

#### 1.3 Analysis Functions Testing
**New File:** `tests/math/test_functions.cpp`
- Test compile-time power function
- Test with different types (int, float, double)
- Test with different exponents (0, 1, 2, 3, etc.)
- Test constexpr evaluation
- **Estimated:** 8-10 tests

### Phase 2: Expand Existing Coverage (Estimated: 3-4 sessions)

#### 2.1 Square Matrix Decompositions
**New File:** `tests/math/test_square_matrix_decompositions.cpp`
- Create dedicated test file for decomposition algorithms
- Use **Parameterized Tests** for testing with different matrix sizes and properties
  - Example: `TEST_P(SquareMatrixDecompositions, householderQR__Success)`
  - Parameters: matrix size, matrix type (well-conditioned, ill-conditioned, symmetric, etc.)
- Use **Typed Tests** for testing with different value types (float32, float64) and storage layouts
- Test naming convention: `<decomposition>__<expected_result>`
  - Examples: `householderQR__Success`, `decomposeLLT_NotSymmetric__ExpectError`
- Test Householder QR decomposition
  - Test with ill-conditioned matrices
  - Test orthogonality of Q: `householderQR_OrthogonalityOfQ__Success`
  - Test upper triangular property of R: `householderQR_UpperTriangularR__Success`
  - Test reconstruction (Q*R = A): `householderQR_Reconstruction__Success`
  - Test with different matrix sizes (use parameterized tests)
- Test LLT decomposition
  - Test with various positive definite matrices
  - Test error handling: `decomposeLLT_NotSymmetric__ExpectError`
  - Test error handling: `decomposeLLT_NotPositiveDefinite__ExpectError`
  - Test reconstruction (L*L' = A): `decomposeLLT_Reconstruction__Success`
- Test LDLT decomposition
  - Test reconstruction (L*D*L' = A): `decomposeLDLT_Reconstruction__Success`
  - Test with various symmetric matrices (use parameterized tests)
  - Test unit diagonal property of L: `decomposeLDLT_UnitDiagonalL__Success`
- Test UDUT decomposition
  - Test reconstruction (U*D*U' = A): `decomposeUDUT_Reconstruction__Success`
  - Test numerical stability: `decomposeUDUT_NumericalStability__Success`
  - Test unit diagonal property of U: `decomposeUDUT_UnitDiagonalU__Success`
- **Estimated:** 20-25 tests

#### 2.2 Vector Operations
**Expand:** [`test_vector.cpp`](tests/math/test_vector.cpp)
- Follow existing naming convention: `<operation>__<expected_result>`
- Use **Typed Tests** if testing with different value types
- Add tests for vector arithmetic operations
  - Examples: `op_plus__Success`, `op_minus__Success`, `op_mul_scalar__Success`
- Add tests for cross product (if implemented): `crossProduct__Success`
- Add tests for vector-matrix multiplication: `op_mul_matrix__Success`
- Add tests for element-wise operations: `elementWise_multiply__Success`
- Add tests for vector views: `VectorView_operations__Success`
- Add edge cases: `ZeroVector_norm__Success`, `UnitVector_normalize__Success`
- **Estimated:** 10-15 additional tests

#### 2.3 Covariance Matrix Operations
**Expand:** [`test_covariance_matrix_full.cpp`](tests/math/test_covariance_matrix_full.cpp), [`test_covariance_matrix_factored.cpp`](tests/math/test_covariance_matrix_factored.cpp)
- Add tests for symmetry preservation
- Add tests for positive semi-definiteness
- Add tests for conversion between full and factored forms
- Add tests for Thornton update algorithm
- Add tests for numerical stability comparisons
- Add tests for large matrices
- **Estimated:** 15-20 additional tests

#### 2.4 Triangular Matrix Operations
**Expand:** [`test_triangular_matrix.cpp`](tests/math/test_triangular_matrix.cpp)
- Add tests for triangular solve operations
- Add tests for forward/backward substitution
- Add tests for triangular matrix inversion
- Add tests for determinant calculation
- Add edge cases (singular matrices, near-singular)
- **Estimated:** 10-15 additional tests

#### 2.5 Diagonal Matrix Operations
**Expand:** [`test_diagonal_matrix.cpp`](tests/math/test_diagonal_matrix.cpp)
- Add tests for diagonal matrix eigenvalues
- Add tests for diagonal matrix powers
- Add tests for diagonal matrix exponential
- Add tests for condition number
- **Estimated:** 8-10 additional tests

### Phase 3: Integration and Edge Cases (Estimated: 2-3 sessions)

#### 3.1 Cross-Class Integration Tests
**New File:** `tests/math/test_matrix_integration.cpp`
- Test interactions between different matrix types
- Use **Typed Tests** for testing with different storage layouts
- Test naming convention: `<operation>_<types>__<expected_result>`
  - Examples: `ConversionChain_MatrixToSquareToDiagonal__Success`, `MixedOp_DiagonalTimesTriangular__Success`
- Test conversion chains: `ConversionChain_MatrixToSquareToDiagonal__Success`
- Test mixed operations: `MixedOp_DiagonalTimesTriangular__Success`
- Test view operations: `ViewOp_AcrossDifferentMatrixTypes__Success`
- **Estimated:** 15-20 tests

#### 3.2 Numerical Stability Tests
**New File:** `tests/math/test_numerical_stability.cpp`
- Use **Parameterized Tests** for different condition numbers and matrix properties
- Test naming convention: `<operation>_<condition>__<expected_result>`
  - Examples: `IllConditioned_decomposition__Success`, `NearSingular_inverse__Success`
- Test ill-conditioned matrices: `IllConditioned_decomposition__Success`
- Test near-singular matrices: `NearSingular_inverse__Success`
- Test very large and very small values: `ExtremeValues_operations__Success`
- Compare factored vs full covariance: `FactoredVsFull_stability__Success`
- Test accumulation of rounding errors: `RoundingErrors_accumulation__Success`
- **Estimated:** 10-15 tests

#### 3.3 Performance Benchmarks (Optional)
**New File:** `tests/math/benchmark_matrix_operations.cpp`
- Benchmark matrix multiplication
- Benchmark decompositions
- Benchmark factored vs full covariance operations
- Compare with Eigen (if available)
- **Note:** This is optional and for development purposes

### Phase 4: Error Handling and Edge Cases (Estimated: 1-2 sessions)

#### 4.1 Error Handling Tests
**Expand:** Multiple test files
- Test all error paths in tl::expected returns
- Test boundary conditions
- Test invalid inputs
- Test dimension mismatches
- Test division by zero
- Test access out of bounds
- **Estimated:** 20-25 additional tests across files

## Implementation Strategy

### Documentation Implementation

1. **Tooling Setup:**
   - Ensure Doxygen is properly configured (already done via [`Doxyfile`](Doxyfile))
   - Set up documentation generation workflow
   - Consider adding documentation coverage reporting

2. **Documentation Standards:**
   - Use Doxygen triple-slash comments (`///`) for all documentation
   - Include `\brief` for all classes and functions (backslash style, not @)
   - Include `\tparam` for all template parameters (backslash style, not @)
   - Include `\param[in]` or `\param[out]` for all function parameters (backslash style, not @)
   - Include `\return` for all return values (backslash style, not @)
   - Include `\note` for important implementation details (backslash style, not @)
   - Include `\warning` for usage warnings (backslash style, not @)
   - Include `\see` for cross-references (backslash style, not @)
   - Include mathematical notation using LaTeX where appropriate
   - Follow the style used in [`matrix.h`](include/trackingLib/math/linalg/matrix.h) as reference

3. **Documentation Review:**
   - Generate Doxygen documentation after each phase
   - Review generated HTML for completeness
   - Check for broken links and missing cross-references
   - Verify code examples compile

### Test Implementation

1. **Test Organization:**
   - Follow existing test structure in [`tests/math/`](tests/math/)
   - Use GoogleTest framework
   - Use typed tests for template instantiations
   - Use parameterized tests for multiple configurations
   - Disable clang-format locally for matrix definitions

2. **Test Standards:**
   - Follow naming convention from [`test_matrix.cpp`](tests/math/test_matrix.cpp): `<operation>__<expected_result>`
     - Examples: `ctor_Zeros__Success`, `op_at__FailBadRowIdx`, `FromList_TooFewRows__ThrowsRuntimeError`
   - Use `// NOLINT` to suppress clang-tidy warnings on test function names
   - Use **Typed Tests** (`TYPED_TEST`) when testing template classes with different template parameters
     - Example: Testing both row-major and column-major matrix layouts
     - Create helper struct to wrap template parameters (e.g., `MatrixStorageType<bool IsRowMajor_>`)
     - Use `TYPED_TEST_SUITE(TestClass, TypeList)` to define type list
   - Use **Parameterized Tests** (`TEST_P`) when testing same functionality with different input values
     - Example: Testing decompositions with different matrix sizes or properties
     - Use `INSTANTIATE_TEST_SUITE_P` to provide parameter values
   - Include both positive and negative test cases
   - Test edge cases and boundary conditions
   - Test error handling paths
   - Use meaningful assertions with clear failure messages
   - Disable clang-format locally for matrix definitions to improve readability

3. **Coverage Measurement:**
   - Use lcov for coverage reporting (already configured)
   - Run coverage build after each phase
   - Target >90% line coverage for math layer
   - Target >85% branch coverage for math layer

4. **Test Review:**
   - Ensure all tests pass before committing
   - Review coverage reports
   - Identify remaining gaps
   - Add tests for uncovered code paths

## Success Criteria

### Documentation Success Criteria

1. **Coverage Metrics:**
   - All public classes have class-level documentation
   - All public methods have function-level documentation
   - All template parameters are documented
   - All parameters and return values are documented
   - All error conditions are documented

2. **Quality Metrics:**
   - Documentation is clear and understandable
   - Mathematical notation is used where appropriate
   - Usage examples are provided for complex operations
   - Cross-references are complete and accurate
   - Academic references are included where relevant

3. **Doxygen Output:**
   - Doxygen generates without warnings
   - All classes appear in class hierarchy
   - All files appear in file list
   - Call graphs are complete
   - Include graphs are complete

### Test Coverage Success Criteria

1. **Coverage Metrics:**
   - Line coverage >90% for math layer
   - Branch coverage >85% for math layer
   - All public methods have at least one test
   - All error paths are tested

2. **Test Quality:**
   - All tests pass consistently
   - Tests are independent and can run in any order
   - Tests have clear names and purposes
   - Tests cover edge cases and boundary conditions
   - Tests verify both success and failure paths

3. **Test Count:**
   - Target: 310+ total tests for math layer (currently 215)
   - New tests: ~95-110 additional tests
   - Distribution:
     - Conversions: 30-40 tests
     - Modified Gram-Schmidt: 10-15 tests
     - Analysis functions: 8-10 tests
     - Square matrix decompositions: 20-25 tests
     - Expanded coverage: 35-45 tests
     - Integration tests: 15-20 tests
     - Numerical stability: 10-15 tests
     - Error handling: 20-25 tests

## Risks and Mitigation

### Risks

1. **Time Investment:** Documentation and testing are time-consuming
   - **Mitigation:** Prioritize high-impact areas first, implement in phases

2. **Maintenance Burden:** Documentation can become outdated
   - **Mitigation:** Integrate documentation review into code review process

3. **Test Complexity:** Some algorithms are complex to test
   - **Mitigation:** Use reference implementations (Eigen) for comparison

4. **Coverage Gaps:** May discover additional gaps during implementation
   - **Mitigation:** Iterative approach, update plan as needed

### Dependencies

1. **Doxygen:** Must be installed and configured (already done)
2. **GoogleTest:** Already integrated via CMake
3. **lcov:** Already configured for coverage reporting
4. **Eigen:** Optional, for reference comparisons in tests

## Timeline Estimate

### Documentation Phases
- Phase 1 (Core Classes): 3-4 sessions
- Phase 2 (New Files): 2-3 sessions
- Phase 3 (View Classes): 2 sessions
- Phase 4 (Covariance): 2 sessions
- Phase 5 (Specialized Ops): 2 sessions
- Phase 6 (Utility): 1 session
- Phase 7 (Contracts): 1 session
- **Total Documentation:** 13-17 sessions

### Testing Phases
- Phase 1 (Critical Gaps): 3-4 sessions
- Phase 2 (Expand Coverage): 3-4 sessions
- Phase 3 (Integration): 2-3 sessions
- Phase 4 (Error Handling): 1-2 sessions
- **Total Testing:** 9-13 sessions

### Overall Timeline
- **Total Estimated Sessions:** 22-30 sessions
- **Recommended Approach:** Interleave documentation and testing phases
- **Suggested Order:** 
  1. Document and test conversions (high priority, recently added)
  2. Document and test decompositions (high priority, complex)
  3. Document and expand core class tests (medium priority)
  4. Document and test specialized operations (medium priority)
  5. Document and test utilities (low priority)

## Next Steps

1. **Review this plan** with the user to confirm priorities and approach
2. **Adjust timeline** based on available resources and urgency
3. **Begin with Phase 1** of documentation (core classes) or Phase 1 of testing (conversions)
4. **Iterate and refine** the plan based on discoveries during implementation
5. **Track progress** using the todo list and update memory bank as needed

## Questions for User

1. **Priority:** Should we prioritize documentation or testing first, or interleave them?
2. **Scope:** Are there specific files or areas that are more critical than others?
3. **Timeline:** What is the desired timeline for completion?
4. **Resources:** Will this be done by one person or multiple contributors?
5. **Standards:** Are there any specific documentation or testing standards beyond what's outlined?
6. **Coverage Targets:** Are the proposed coverage targets (>90% line, >85% branch) acceptable?
7. **Academic References:** Should we include extensive academic references in documentation?
8. **Examples:** How many usage examples should be included per class/function?

## Appendix: File Inventory

### Files Requiring Documentation (Priority Order)

**High Priority (Recently Added/Modified):**
1. [`matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h)
2. [`conversions/conversions.h`](include/trackingLib/math/linalg/conversions/conversions.h)
3. [`conversions/matrix_conversions.hpp`](include/trackingLib/math/linalg/conversions/matrix_conversions.hpp)
4. [`conversions/diagonal_conversions.hpp`](include/trackingLib/math/linalg/conversions/diagonal_conversions.hpp)
5. [`conversions/square_conversions.hpp`](include/trackingLib/math/linalg/conversions/square_conversions.hpp)
6. [`conversions/triangular_conversions.hpp`](include/trackingLib/math/linalg/conversions/triangular_conversions.hpp)
7. [`conversions/vector_conversions.hpp`](include/trackingLib/math/linalg/conversions/vector_conversions.hpp)
8. [`conversions/covariance_matrix_conversions.hpp`](include/trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp)
9. [`square_matrix_decompositions.hpp`](include/trackingLib/math/linalg/square_matrix_decompositions.hpp)

**Medium Priority (Core Classes):**
10. [`matrix.h`](include/trackingLib/math/linalg/matrix.h)
11. [`square_matrix.h`](include/trackingLib/math/linalg/square_matrix.h)
12. [`triangular_matrix.h`](include/trackingLib/math/linalg/triangular_matrix.h)
13. [`diagonal_matrix.h`](include/trackingLib/math/linalg/diagonal_matrix.h)
14. [`vector.h`](include/trackingLib/math/linalg/vector.h)
15. [`covariance_matrix_full.h`](include/trackingLib/math/linalg/covariance_matrix_full.h)
16. [`covariance_matrix_factored.h`](include/trackingLib/math/linalg/covariance_matrix_factored.h)

**Medium Priority (Views and Specialized):**
17. [`matrix_view.h`](include/trackingLib/math/linalg/matrix_view.h)
18. [`matrix_row_view.h`](include/trackingLib/math/linalg/matrix_row_view.h)
19. [`matrix_column_view.h`](include/trackingLib/math/linalg/matrix_column_view.h)
20. [`rank1_update.h`](include/trackingLib/math/linalg/rank1_update.h)
21. [`modified_gram_schmidt.h`](include/trackingLib/math/linalg/modified_gram_schmidt.h)

**Low Priority (Utilities):**
22. [`matrix_types.h`](include/trackingLib/math/linalg/matrix_types.h)
23. [`errors.h`](include/trackingLib/math/linalg/errors.h)
24. [`point2d.h`](include/trackingLib/math/linalg/point2d.h)
25. [`point3d.h`](include/trackingLib/math/linalg/point3d.h)
26. [`functions.h`](include/trackingLib/math/analysis/functions.h)
27. [`contracts/matrix_intf.h`](include/trackingLib/math/linalg/contracts/matrix_intf.h)
28. [`contracts/covariance_matrix_intf.h`](include/trackingLib/math/linalg/contracts/covariance_matrix_intf.h)

### Test Files to Create/Expand

**New Test Files Needed:**
1. `tests/math/test_conversions.cpp` (30-40 tests)
2. `tests/math/test_modified_gram_schmidt.cpp` (10-15 tests)
3. `tests/math/test_functions.cpp` (8-10 tests)
4. `tests/math/test_square_matrix_decompositions.cpp` (20-25 tests)
5. `tests/math/test_matrix_integration.cpp` (15-20 tests)
6. `tests/math/test_numerical_stability.cpp` (10-15 tests)

**Existing Test Files to Expand:**
1. [`tests/math/test_square_matrix.cpp`](tests/math/test_square_matrix.cpp) (+5-8 tests for non-decomposition methods)
2. [`tests/math/test_vector.cpp`](tests/math/test_vector.cpp) (+10-15 tests)
3. [`tests/math/test_covariance_matrix_full.cpp`](tests/math/test_covariance_matrix_full.cpp) (+8-10 tests)
4. [`tests/math/test_covariance_matrix_factored.cpp`](tests/math/test_covariance_matrix_factored.cpp) (+8-10 tests)
5. [`tests/math/test_triangular_matrix.cpp`](tests/math/test_triangular_matrix.cpp) (+10-15 tests)
6. [`tests/math/test_diagonal_matrix.cpp`](tests/math/test_diagonal_matrix.cpp) (+8-10 tests)
7. All test files for error handling (+20-25 tests distributed)
