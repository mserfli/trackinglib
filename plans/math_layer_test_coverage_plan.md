# Math Layer Test Coverage Plan

## Executive Summary

This plan addresses the need to analyze and improve test coverage for the math layer of trackinglib. Recent refactoring efforts have resulted in new files being added and some unittest code being partially removed. This analysis will ensure comprehensive test coverage across all math layer components.

## Current State Analysis

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

## Test Coverage Improvement Plan

### Phase 1: Critical Gaps - New Files (Estimated: 3-4 sessions)

#### 1.1 Conversion Functions Testing  (COMPLETED 2026-01-02)
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

#### 1.2 Modified Gram-Schmidt Testing (COMPLETED 2026-01-05)
**New File:** `tests/math/test_modified_gram_schmidt.cpp` (6 test cases)
- ✅ Test MGS algorithm for UDU factorization
- ✅ Use **Parameterized Tests** for different matrix sizes (2, 4, 6)
- ✅ Test naming convention: `run_<transformation>__<expected_result>`
  - Examples: `run_PhiUDUPhiT__Success`, `run_PhiUDUPhiT_PlusGQGT__Success`
- ✅ Test `Phi*UDU'*Phi'` transformation: `run_PhiUDUPhiT__Success`
- ✅ Test `Phi*UDU'*Phi' + G*Q*G'` transformation: `run_PhiUDUPhiT_PlusGQGT__Success`
- ✅ Test with different matrix sizes (parameterized tests for 2, 4, 6)
- ✅ Test numerical stability: `run_NumericalStability__Success`
- ✅ Test edge cases: `run_IdentityPhi__Success`, `run_ZeroProcessNoise__Success`
- ✅ Compare results with manually computed reference implementations
- **Actual:** 6 comprehensive test cases covering all critical functionality
- **Coverage:** Excellent - both core methods and edge cases thoroughly tested

#### 1.3 Analysis Functions Testing (COMPLETED 2026-01-05)
**New File:** `tests/math/test_functions.cpp` (22 test cases)
- ✅ Test compile-time power function
- ✅ Test with different types (int, float, double)
- ✅ Test with different exponents (0, 1, 2, 3, 4, 5, 10)
- ✅ Test constexpr evaluation capabilities
- ✅ Test edge cases (negative bases, zero base, one base)
- ✅ Test template parameter usage
- ✅ Test large exponents
- **Actual:** 22 comprehensive test cases covering all functionality
- **Coverage:** Excellent - all code paths thoroughly tested
- **Test Count:** Increased from 229 to 251 total tests

### Phase 2: Expand Existing Coverage (Estimated: 3-4 sessions)

#### 2.1 Square Matrix Decompositions (COMPLETED 2026-01-05)
**New File:** `tests/math/test_square_matrix_decompositions.cpp` (22 tests)
- ✅ Created dedicated test file for decomposition algorithms
- ✅ Implemented comprehensive tests for all four decomposition algorithms
- ✅ Test naming convention: `<decomposition>__<expected_result>`
  - Examples: `householderQR__Success`, `decomposeLLT_NotSymmetric_ExpectError`
- ✅ Householder QR decomposition tests:
  - `householderQR__Success`: Basic functionality test
  - `householderQR_OrthogonalityOfQ__Success`: Tests Q^T * Q = I property
  - `householderQR_UpperTriangularR__Success`: Tests R is upper triangular
  - `householderQR_Reconstruction__Success`: Tests Q * R = original matrix
  - `householderQR_Double__Success`: Tests with double precision
- ✅ LLT decomposition tests:
  - `decomposeLLT__Success`: Basic functionality test
  - `decomposeLLT_Reconstruction__Success`: Tests L * L^T = original matrix
  - `decomposeLLT_NotSymmetric_ExpectError`: Tests error handling for non-symmetric matrices
  - `decomposeLLT_SymmetricNotPositiveDefinite_ExpectError`: Tests error handling for non-positive definite matrices
  - `decomposeLLT_Double__Success`: Tests with double precision
- ✅ LDLT decomposition tests:
  - `decomposeLDLT__Success`: Basic functionality test
  - `decomposeLDLT_Reconstruction__Success`: Tests (L * D) * L^T = original matrix
  - `decomposeLDLT_UnitDiagonalL__Success`: Tests L has unit diagonal
  - `decomposeLDLT_NotSymmetric_ExpectError`: Tests error handling for non-symmetric matrices
  - `decomposeLDLT_SymmetricNotPositiveDefinite_ExpectError`: Tests error handling for non-positive definite matrices
  - `decomposeLDLT_Double__Success`: Tests with double precision
- ✅ UDUT decomposition tests:
  - `decomposeUDUT__Success`: Basic functionality test
  - `decomposeUDUT_Reconstruction__Success`: Tests (U * D) * U^T = original matrix
  - `decomposeUDUT_UnitDiagonalU__Success`: Tests U has unit diagonal
  - `decomposeUDUT_NumericalStability__Success`: Tests with ill-conditioned matrices
  - `decomposeUDUT_NotSymmetric_ExpectError`: Tests error handling for non-symmetric matrices
  - `decomposeUDUT_Double__Success`: Tests with double precision
- ✅ Added helper functions for test validation:
  - `isOrthogonal()`: Checks if matrix is orthogonal
  - `isUpperTriangular()`: Checks if matrix is upper triangular
  - `isLowerTriangular()`: Checks if matrix is lower triangular
  - `hasUnitDiagonal()`: Checks if matrix has unit diagonal
  - `createSymmetricPositiveDefiniteMatrix()`: Creates test matrices
  - `createIllConditionedMatrix()`: Creates ill-conditioned test matrices
- ✅ All tests pass (22/22)
- ✅ Total test count increased from 251 to 273
- **Actual:** 22 comprehensive tests covering all decomposition algorithms
- **Coverage:** Excellent - all decomposition methods thoroughly tested with both float32 and float64 types

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
     - Target: 310+ total tests for math layer (currently 273)
     - New tests: ~37-53 additional tests remaining
     - Distribution:
       - Conversions: 30-40 tests (COMPLETED: 30-40 tests)
       - Modified Gram-Schmidt: 10-15 tests (COMPLETED: 6 tests)
       - Analysis functions: 8-10 tests (COMPLETED: 22 tests)
       - Square matrix decompositions: 20-25 tests (COMPLETED: 22 tests)
       - Expanded coverage: 35-45 tests (PENDING)
       - Integration tests: 15-20 tests (PENDING)
       - Numerical stability: 10-15 tests (PENDING)
       - Error handling: 20-25 tests (PENDING)

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

### Testing Phases
- Phase 1 (Critical Gaps): 3-4 sessions
- Phase 2 (Expand Coverage): 3-4 sessions
- Phase 3 (Integration): 2-3 sessions
- Phase 4 (Error Handling): 1-2 sessions
- **Total Testing:** 9-13 sessions

### Overall Timeline
- **Total Estimated Sessions:** 9-13 sessions
- **Recommended Approach:** Focus on testing phases
- **Suggested Order:** 
  1. Test conversions (high priority, recently added)
  2. Test decompositions (high priority, complex)
  3. Expand core class tests (medium priority)
  4. Test specialized operations (medium priority)
  5. Test utilities (low priority)

## Next Steps

1. **Review this plan** with the user to confirm priorities and approach
2. **Adjust timeline** based on available resources and urgency
3. **Continue with Phase 2** of testing (expanded coverage)
4. **Iterate and refine** the plan based on discoveries during implementation
5. **Track progress** using the todo list and update memory bank as needed

## Questions for User

1. **Priority:** Should we prioritize testing first?
2. **Scope:** Are there specific files or areas that are more critical than others?
3. **Timeline:** What is the desired timeline for completion?
4. **Resources:** Will this be done by one person or multiple contributors?
5. **Coverage Targets:** Are the proposed coverage targets (>90% line, >85% branch) acceptable?
6. **Reference Implementations:** Should we use Eigen for comparison in tests?

## Appendix: Test Files to Create/Expand

### New Test Files Needed:
1. `tests/math/test_conversions.cpp` (30-40 tests)
2. `tests/math/test_modified_gram_schmidt.cpp` (10-15 tests)
3. `tests/math/test_functions.cpp` (8-10 tests)
4. `tests/math/test_square_matrix_decompositions.cpp` (20-25 tests)
5. `tests/math/test_matrix_integration.cpp` (15-20 tests)
6. `tests/math/test_numerical_stability.cpp` (10-15 tests)

### Existing Test Files to Expand:
1. [`tests/math/test_square_matrix.cpp`](tests/math/test_square_matrix.cpp) (+5-8 tests for non-decomposition methods)
2. [`tests/math/test_vector.cpp`](tests/math/test_vector.cpp) (+10-15 tests)
3. [`tests/math/test_covariance_matrix_full.cpp`](tests/math/test_covariance_matrix_full.cpp) (+8-10 tests)
4. [`tests/math/test_covariance_matrix_factored.cpp`](tests/math/test_covariance_matrix_factored.cpp) (+8-10 tests)
5. [`tests/math/test_triangular_matrix.cpp`](tests/math/test_triangular_matrix.cpp) (+10-15 tests)
6. [`tests/math/test_diagonal_matrix.cpp`](tests/math/test_diagonal_matrix.cpp) (+8-10 tests)
7. All test files for error handling (+20-25 tests distributed)