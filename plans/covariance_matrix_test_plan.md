# Covariance Matrix Operations Test Plan

## Executive Summary

This plan addresses section 2.3 of the math layer test coverage plan, focusing on expanding test coverage for covariance matrix operations. The goal is to add 15-20 comprehensive tests covering critical functionality that is currently missing.

## Current State Analysis

### Existing Test Coverage

**CovarianceMatrixFull (6 tests):**
- `compose`: Test basic composition
- `apaT`: Test A*P*A^T operation (in-place and const versions)
- `apaT_equality`: Test mathematical properties of apaT
- `inverse_const`: Test matrix inversion
- `setVariance`: Test variance setting

**CovarianceMatrixFactored (8 tests):**
- `ctor_from_full_matrix`: Test construction from full matrix
- `compose`: Test composition to full matrix
- `calcCovarianceElement`: Parameterized test for element access
- `apaT`: Test A*P*A^T operation (in-place and const versions)
- `rank1Update_upper`: Test rank-1 update
- `setVariance`: Test variance setting

### Missing Test Coverage

1. **Symmetry Preservation**: No tests verify that covariance matrices remain symmetric after operations
2. **Positive Semi-Definiteness**: No tests verify this fundamental property
3. **Conversion Operations**: No tests for conversion between full and factored forms
4. **Thornton Update**: No tests for the Thornton prediction algorithm
5. **Numerical Stability**: No comparative tests between full and factored forms
6. **Large Matrices**: No tests for larger matrix sizes
7. **Edge Cases**: Limited testing of edge cases and error conditions

## Test Implementation Plan

### 1. Symmetry Preservation Tests

**Objective**: Verify that all operations preserve the symmetry property P[i,j] = P[j,i]

**Tests to Add:**
- `symmetry_preservation_apaT__Success` (both full and factored)
- `symmetry_preservation_rank1Update__Success` (factored only)
- `symmetry_preservation_thornton__Success` (factored only)
- `symmetry_preservation_setVariance__Success` (both)

**Implementation:**
- Create helper function `isSymmetric()` to check matrix symmetry
- Test symmetry after each major operation
- Use parameterized tests for different matrix sizes

### 2. Positive Semi-Definiteness Tests

**Objective**: Verify that covariance matrices remain positive semi-definite

**Tests to Add:**
- `positive_semi_definite_apaT__Success` (both)
- `positive_semi_definite_rank1Update__Success` (factored)
- `positive_semi_definite_thornton__Success` (factored)
- `positive_semi_definite_inverse__Success` (both)

**Implementation:**
- Create helper function `isPositiveSemiDefinite()` using eigenvalue analysis
- Test PSD property after operations that could violate it
- Include edge cases (near-singular matrices)

### 3. Conversion Between Full and Factored Forms

**Objective**: Test bidirectional conversion and equivalence

**Tests to Add:**
- `conversion_full_to_factored__Success`
- `conversion_factored_to_full__Success`
- `conversion_roundtrip_equivalence__Success`
- `conversion_preserves_properties__Success`

**Implementation:**
- Test conversion functions from conversion headers
- Verify mathematical equivalence after round-trip conversion
- Test property preservation (symmetry, PSD)

### 4. Thornton Update Algorithm Tests

**Objective**: Comprehensive testing of Thornton's prediction algorithm

**Tests to Add:**
- `thornton_basic__Success`
- `thornton_with_process_noise__Success`
- `thornton_numerical_stability__Success`
- `thornton_large_matrices__Success`
- `thornton_equivalence_with_full__Success`

**Implementation:**
- Test with different matrix sizes (3x3, 4x4, 6x6)
- Compare results with full matrix implementation
- Test numerical stability with ill-conditioned matrices

### 5. Numerical Stability Comparisons

**Objective**: Compare numerical stability between full and factored forms

**Tests to Add:**
- `numerical_stability_ill_conditioned__Success`
- `numerical_stability_near_singular__Success`
- `numerical_stability_extreme_values__Success`
- `numerical_stability_accumulation__Success`

**Implementation:**
- Create ill-conditioned test matrices
- Compare results between full and factored implementations
- Test with extreme value ranges

### 6. Large Matrix Tests

**Objective**: Test with larger matrix sizes

**Tests to Add:**
- `large_matrix_apaT__Success` (6x6, 8x8)
- `large_matrix_thornton__Success` (6x6)
- `large_matrix_conversion__Success`

**Implementation:**
- Test with matrix sizes up to 8x8
- Verify performance and correctness
- Test memory usage patterns

## Detailed Test Specifications

### CovarianceMatrixFull Tests (Add ~8-10 tests)

```cpp
// Symmetry preservation
TEST(CovarianceMatrixFull, symmetry_preservation_apaT__Success)
TEST(CovarianceMatrixFull, symmetry_preservation_setVariance__Success)

// Positive semi-definiteness
TEST(CovarianceMatrixFull, positive_semi_definite_apaT__Success)
TEST(CovarianceMatrixFull, positive_semi_definite_inverse__Success)

// Conversion operations
TEST(CovarianceMatrixFull, conversion_to_factored__Success)
TEST(CovarianceMatrixFull, conversion_roundtrip__Success)

// Large matrices
TEST(CovarianceMatrixFull, large_matrix_apaT__Success)
TEST(CovarianceMatrixFull, large_matrix_inverse__Success)
```

### CovarianceMatrixFactored Tests (Add ~10-12 tests)

```cpp
// Symmetry preservation
TEST(CovarianceMatrixFactored, symmetry_preservation_apaT__Success)
TEST(CovarianceMatrixFactored, symmetry_preservation_rank1Update__Success)
TEST(CovarianceMatrixFactored, symmetry_preservation_thornton__Success)

// Positive semi-definiteness
TEST(CovarianceMatrixFactored, positive_semi_definite_rank1Update__Success)
TEST(CovarianceMatrixFactored, positive_semi_definite_thornton__Success)

// Thornton update algorithm
TEST(CovarianceMatrixFactored, thornton_basic__Success)
TEST(CovarianceMatrixFactored, thornton_with_process_noise__Success)
TEST(CovarianceMatrixFactored, thornton_numerical_stability__Success)
TEST(CovarianceMatrixFactored, thornton_large_matrices__Success)

// Conversion operations
TEST(CovarianceMatrixFactored, conversion_to_full__Success)
TEST(CovarianceMatrixFactored, conversion_roundtrip__Success)
```

## Helper Functions Needed

### Matrix Property Helpers

```cpp
// Check if matrix is symmetric
template <typename MatrixType>
bool isSymmetric(const MatrixType& mat, typename MatrixType::value_type tolerance = 1e-6)
{
  for (sint32 i = 0; i < MatrixType::dim; ++i) {
    for (sint32 j = i + 1; j < MatrixType::dim; ++j) {
      if (std::abs(mat(i, j) - mat(j, i)) > tolerance) {
        return false;
      }
    }
  }
  return true;
}

// Check if matrix is positive semi-definite (using eigenvalue analysis)
template <typename MatrixType>
bool isPositiveSemiDefinite(const MatrixType& mat, typename MatrixType::value_type tolerance = 1e-6)
{
  // Implement using Cholesky decomposition or eigenvalue analysis
  // Return true if all eigenvalues >= -tolerance
}
```

### Test Data Generators

```cpp
// Create symmetric positive definite matrix
template <typename FloatType, sint32 Size>
auto createSymmetricPositiveDefiniteMatrix() -> CovarianceMatrixFull<FloatType, Size>
{
  // Create a matrix guaranteed to be symmetric and positive definite
}

// Create ill-conditioned matrix
template <typename FloatType, sint32 Size>
auto createIllConditionedMatrix() -> CovarianceMatrixFull<FloatType, Size>
{
  // Create matrix with high condition number
}
```

## Implementation Strategy

### Phase 1: Helper Functions (1 session)
1. Implement matrix property helper functions
2. Implement test data generators
3. Add to existing test files or create helper header

### Phase 2: Core Functionality Tests (2 sessions)
1. Implement symmetry preservation tests
2. Implement positive semi-definiteness tests
3. Implement conversion tests
4. Test with both float32 and float64 types

### Phase 3: Advanced Algorithm Tests (2 sessions)
1. Implement Thornton update tests
2. Implement numerical stability comparisons
3. Implement large matrix tests
4. Add edge case testing

### Phase 4: Review and Finalization (1 session)
1. Run all tests and verify coverage
2. Fix any issues discovered
3. Update documentation
4. Finalize test count (target: 15-20 new tests)

## Success Criteria

### Test Coverage Metrics
- **CovarianceMatrixFull**: Increase from 6 to 14-16 tests
- **CovarianceMatrixFactored**: Increase from 8 to 18-20 tests
- **Total New Tests**: 15-20 comprehensive tests
- **Coverage Improvement**: Significant increase in line and branch coverage

### Quality Standards
- All tests follow existing naming conventions
- Tests cover both success and edge cases
- Tests use appropriate GoogleTest patterns (Typed Tests, Parameterized Tests)
- Tests include clear assertions with meaningful failure messages
- Tests maintain consistency with existing code style

## Risks and Mitigation

### Risks
1. **Numerical Stability Testing Complexity**: Creating meaningful ill-conditioned test cases
   - *Mitigation*: Use academic references for test matrix generation

2. **Thornton Algorithm Complexity**: Understanding and testing the complex algorithm
   - *Mitigation*: Compare with full matrix implementation for verification

3. **Performance Impact**: Large matrix tests may be slow
   - *Mitigation*: Limit large matrix sizes to reasonable bounds (6x6, 8x8)

4. **Test Maintenance**: Complex test cases may be fragile
   - *Mitigation*: Use clear, well-documented test data and assertions

## Timeline Estimate

- **Helper Functions**: 1 session
- **Core Tests**: 2 sessions  
- **Advanced Tests**: 2 sessions
- **Review/Finalization**: 1 session
- **Total**: 6 sessions

## Next Steps

1. **Implement helper functions** for matrix property testing
2. **Add symmetry preservation tests** to both covariance matrix test files
3. **Add positive semi-definiteness tests** to both covariance matrix test files
4. **Implement conversion tests** between full and factored forms
5. **Add Thornton update algorithm tests** to factored covariance tests
6. **Add numerical stability comparison tests**
7. **Add large matrix tests** for both implementations
8. **Review and finalize** all new tests

## Questions for User

1. **Priority**: Should we focus on specific aspects first (e.g., Thornton algorithm)?
2. **Test Data**: Are there specific test matrices or scenarios that should be included?
3. **Numerical Tolerances**: Should we use specific tolerance values for comparisons?
4. **Large Matrix Sizes**: What maximum matrix size should we test (6x6, 8x8, larger)?
5. **Reference Implementations**: Should we compare with Eigen or other libraries for verification?