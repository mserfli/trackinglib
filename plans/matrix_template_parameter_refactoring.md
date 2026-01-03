# Matrix Template Parameter Refactoring Plan

## Executive Summary

**Issue Identified**: The current template parameter names `Rows_` and `Cols_` in the [`Matrix`](../include/trackingLib/math/linalg/matrix.h) class are semantically ambiguous because their meaning depends on the `IsRowMajor_` parameter. This creates confusion in documentation and understanding of the code.

**Proposed Solution**: Rename template parameters to `Dim0_` and `Dim1_` to reflect dimension-agnostic semantics, with additional suffixes (`Dim0B_`, `Dim1B_`) for secondary matrix parameters in operations.

**Agreement**: **YES**, this refactoring is necessary and beneficial for code clarity and correctness.

---

## Problem Analysis

### Current Situation

The [`Matrix`](../include/trackingLib/math/linalg/matrix.h:87) class is defined as:
```cpp
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_ = true>
class Matrix
```

However, the actual semantics are:
- When `IsRowMajor_ == true`: `Rows_` = number of rows, `Cols_` = number of columns
- When `IsRowMajor_ == false`: `Rows_` = number of columns, `Cols_` = number of rows (logically)

This is evident from lines 98-99 in [`matrix.h`](../include/trackingLib/math/linalg/matrix.h:98-99):
```cpp
static constexpr auto RowsInMem  = IsRowMajor_ ? Rows_ : Cols_; ///< number of rows in memory
static constexpr auto ColsInMem  = IsRowMajor_ ? Cols_ : Rows_; ///< number of cols in memory
```

### Why This Is Problematic

1. **Semantic Confusion**: The names `Rows_` and `Cols_` imply a fixed semantic meaning (rows and columns), but their actual meaning changes based on storage layout.

2. **Documentation Ambiguity**: Current documentation states "Rows_ The number of rows in the matrix" but this is only true for row-major layout.

3. **Transpose Type Definition**: The transpose type swaps `Rows_` and `Cols_` AND flips `IsRowMajor_`:
   ```cpp
   using transpose_type = Matrix<ValueType_, Cols_, Rows_, !IsRowMajor_>;
   ```
   This works correctly but is conceptually confusing.

4. **Consistency with Implementation**: The internal logic already treats these as dimension indices rather than semantic rows/columns.

### Root Cause

The Matrix class uses a **logical dimension** approach where:
- `Dim0` = first logical dimension (what the user sees as rows)
- `Dim1` = second logical dimension (what the user sees as columns)
- `IsRowMajor` determines how these map to memory layout

The current naming (`Rows_`, `Cols_`) implies a **semantic** approach which conflicts with the implementation.

---

## Proposed Solution

### Primary Template Parameters

Rename the Matrix class template parameters:

**Current**:
```cpp
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_ = true>
class Matrix
```

**Proposed**:
```cpp
template <typename ValueType_, sint32 Dim0_, sint32 Dim1_, bool IsRowMajor_ = true>
class Matrix
```

Where:
- `Dim0_` = First logical dimension (number of rows in the logical matrix)
- `Dim1_` = Second logical dimension (number of columns in the logical matrix)
- `IsRowMajor_` = Storage layout (true = row-major, false = column-major)

### Secondary Template Parameters (for operations)

For operations involving two matrices (e.g., matrix multiplication), use suffixed names:

**Current**:
```cpp
template <sint32 Cols2_, bool IsRowMajor2_>
auto operator*(const Matrix<ValueType_, Cols_, Cols2_, IsRowMajor2_>& other) const
    -> Matrix<ValueType_, Rows_, Cols2_, IsRowMajor_>;
```

**Proposed**:
```cpp
template <sint32 Dim1B_, bool IsRowMajorB_>
auto operator*(const Matrix<ValueType_, Dim1_, Dim1B_, IsRowMajorB_>& other) const
    -> Matrix<ValueType_, Dim0_, Dim1B_, IsRowMajor_>;
```

**Naming Convention for Secondary Parameters**:
- `Dim0B_`, `Dim1B_` = Dimensions of second matrix (B)
- `IsRowMajorB_` = Storage layout of second matrix (B)
- `Dim0C_`, `Dim1C_`, `IsRowMajorC_` = Parameters for third matrix (C) if needed
- Rationale: The 'B' suffix clearly indicates "second matrix" while maintaining consistency across all template parameters

### Internal Constants

Update internal constants to reflect the new naming:

**Current**:
```cpp
static constexpr auto Rows       = Rows_;
static constexpr auto Cols       = Cols_;
static constexpr auto RowsInMem  = IsRowMajor_ ? Rows_ : Cols_;
static constexpr auto ColsInMem  = IsRowMajor_ ? Cols_ : Rows_;
```

**Proposed**:
```cpp
static constexpr auto Dim0       = Dim0_;                       ///< first logical dimension (rows)
static constexpr auto Dim1       = Dim1_;                       ///< second logical dimension (columns)
static constexpr auto RowsInMem  = IsRowMajor_ ? Dim0_ : Dim1_; ///< number of rows in memory
static constexpr auto ColsInMem  = IsRowMajor_ ? Dim1_ : Dim0_; ///< number of columns in memory
```

**Rationale**: Remove `Rows` and `Cols` aliases entirely to eliminate semantic confusion. Users should use `Dim0` and `Dim1` consistently, which makes the dimension-agnostic nature explicit and prevents misunderstandings about what these values represent.

---

## Impact Analysis

### Files Requiring Changes

Based on the search results, the following files use `Rows_`, `Cols_`, `Rows2_`, `Cols2_` template parameters:

#### Core Matrix Files (HIGH PRIORITY)
1. [`include/trackingLib/math/linalg/matrix.h`](../include/trackingLib/math/linalg/matrix.h) - Primary class definition
2. [`include/trackingLib/math/linalg/matrix.hpp`](../include/trackingLib/math/linalg/matrix.hpp) - Implementation
3. [`include/trackingLib/math/linalg/matrix_view.h`](../include/trackingLib/math/linalg/matrix_view.h) - MatrixView class
4. [`include/trackingLib/math/linalg/matrix_view.hpp`](../include/trackingLib/math/linalg/matrix_view.hpp) - Implementation
5. [`include/trackingLib/math/linalg/matrix_row_view.h`](../include/trackingLib/math/linalg/matrix_row_view.h) - MatrixRowView class
6. [`include/trackingLib/math/linalg/matrix_row_view.hpp`](../include/trackingLib/math/linalg/matrix_row_view.hpp) - Implementation
7. [`include/trackingLib/math/linalg/matrix_column_view.h`](../include/trackingLib/math/linalg/matrix_column_view.h) - MatrixColumnView class
8. [`include/trackingLib/math/linalg/matrix_column_view.hpp`](../include/trackingLib/math/linalg/matrix_column_view.hpp) - Implementation

#### Specialized Matrix Types (HIGH PRIORITY)
9. [`include/trackingLib/math/linalg/diagonal_matrix.h`](../include/trackingLib/math/linalg/diagonal_matrix.h) - DiagonalMatrix operations with Matrix
10. [`include/trackingLib/math/linalg/diagonal_matrix.hpp`](../include/trackingLib/math/linalg/diagonal_matrix.hpp) - Implementation
11. [`include/trackingLib/math/linalg/triangular_matrix.h`](../include/trackingLib/math/linalg/triangular_matrix.h) - TriangularMatrix operations
12. [`include/trackingLib/math/linalg/triangular_matrix.hpp`](../include/trackingLib/math/linalg/triangular_matrix.hpp) - Implementation

#### Conversion System (MEDIUM PRIORITY)
13. [`include/trackingLib/math/linalg/conversions/conversions.h`](../include/trackingLib/math/linalg/conversions/conversions.h) - Forward declarations
14. [`include/trackingLib/math/linalg/conversions/matrix_conversions.hpp`](../include/trackingLib/math/linalg/conversions/matrix_conversions.hpp) - Matrix conversions

#### Supporting Files (MEDIUM PRIORITY)
15. [`include/trackingLib/math/linalg/vector.h`](../include/trackingLib/math/linalg/vector.h) - Forward declarations
16. [`include/trackingLib/math/linalg/covariance_matrix_factored.h`](../include/trackingLib/math/linalg/covariance_matrix_factored.h) - Forward declarations
17. [`include/trackingLib/math/linalg/modified_gram_schmidt.h`](../include/trackingLib/math/linalg/modified_gram_schmidt.h) - Forward declarations
18. [`include/trackingLib/math/linalg/contracts/matrix_intf.h`](../include/trackingLib/math/linalg/contracts/matrix_intf.h) - Interface contracts

#### Test Files (LOW PRIORITY - Update after implementation)
19. [`tests/math/test_matrix.cpp`](../tests/math/test_matrix.cpp) - Matrix tests
20. [`tests/math/test_matrix_view.cpp`](../tests/math/test_matrix_view.cpp) - View tests
21. [`tests/math/test_matrix_row_view.cpp`](../tests/math/test_matrix_row_view.cpp) - Row view tests
22. [`tests/math/test_matrix_column_view.cpp`](../tests/math/test_matrix_column_view.cpp) - Column view tests
23. Other test files that instantiate Matrix templates

### Scope of Changes

#### Template Parameter Declarations
- **Pattern**: `template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>`
- **Change to**: `template <typename ValueType_, sint32 Dim0_, sint32 Dim1_, bool IsRowMajor_>`
- **Occurrences**: ~50+ locations across all files

#### Secondary Template Parameters
- **Pattern**: `template <sint32 Cols2_, bool IsRowMajor2_>`
- **Change to**: `template <sint32 Dim1B_, bool IsRowMajorB_>`
- **Pattern**: `template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>`
- **Change to**: `template <sint32 Dim0B_, sint32 Dim1B_, bool IsRowMajorB_>`
- **Occurrences**: ~30+ locations

#### Template Instantiations
- **Pattern**: `Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>`
- **Change to**: `Matrix<ValueType_, Dim0_, Dim1_, IsRowMajor_>`
- **Pattern**: `Matrix<ValueType_, Rows2_, Cols2_, IsRowMajor2_>`
- **Change to**: `Matrix<ValueType_, Dim0B_, Dim1B_, IsRowMajorB_>`
- **Occurrences**: ~100+ locations

#### Internal Constant Usage
- **Pattern**: `mat.Rows`, `mat.Cols`
- **Change to**: `mat.Dim0`, `mat.Dim1`
- **Impact**: All code using these constants must be updated
- **Occurrences**: ~50+ locations across implementation and test files

#### Documentation Updates
- All Doxygen `\tparam` comments for `Rows_` and `Cols_`
- Class-level documentation explaining the dimension semantics
- Usage examples in documentation
- **Occurrences**: ~50+ documentation blocks

---

## Detailed Refactoring Strategy

### Phase 1: Core Matrix Class

1. **Update [`matrix.h`](../include/trackingLib/math/linalg/matrix.h)**
   - Change template parameters: `Rows_` → `Dim0_`, `Cols_` → `Dim1_`
   - Update all member function declarations
   - Update internal constants (add `Dim0`, `Dim1`, keep `Rows`, `Cols` as aliases)
   - Update all Doxygen documentation
   - Update transpose type definition

2. **Update [`matrix.hpp`](../include/trackingLib/math/linalg/matrix.hpp)**
   - Update all template parameter lists in implementations
   - Update all references to `Rows_`, `Cols_` in function bodies
   - Verify all static_assert statements

### Phase 2: View Classes

3. **Update [`matrix_view.h`](../include/trackingLib/math/linalg/matrix_view.h) and [`.hpp`](../include/trackingLib/math/linalg/matrix_view.hpp)**
   - Change template parameters
   - Update forward declarations
   - Update member variables and methods

4. **Update [`matrix_row_view.h`](../include/trackingLib/math/linalg/matrix_row_view.h) and [`.hpp`](../include/trackingLib/math/linalg/matrix_row_view.hpp)**
   - Change template parameters
   - Update operations with secondary matrices (use `Dim0B_`, `Dim1B_`, `IsRowMajorB_`)

5. **Update [`matrix_column_view.h`](../include/trackingLib/math/linalg/matrix_column_view.h) and [`.hpp`](../include/trackingLib/math/linalg/matrix_column_view.hpp)**
   - Change template parameters
   - Update operations with secondary matrices (use `Dim0B_`, `Dim1B_`, `IsRowMajorB_`)

### Phase 3: Specialized Matrix Types

6. **Update [`diagonal_matrix.h`](../include/trackingLib/math/linalg/diagonal_matrix.h) and [`.hpp`](../include/trackingLib/math/linalg/diagonal_matrix.hpp)**
   - Update operations involving Matrix: `operator*` with Matrix
   - Change `Cols_` → `Dim1_`, `IsRowMajor_` → `IsRowMajor_` in multiplication operations
   - Update non-member operators with consistent naming

7. **Update [`triangular_matrix.h`](../include/trackingLib/math/linalg/triangular_matrix.h) and [`.hpp`](../include/trackingLib/math/linalg/triangular_matrix.hpp)**
   - Update `operator*` and `solve` methods
   - Change `Cols_` → `Dim1_`, `IsRowMajor2_` → `IsRowMajorB_` in operations

### Phase 4: Conversion System

8. **Update [`conversions/conversions.h`](../include/trackingLib/math/linalg/conversions/conversions.h)**
   - Update forward declarations

9. **Update [`conversions/matrix_conversions.hpp`](../include/trackingLib/math/linalg/conversions/matrix_conversions.hpp)**
   - Update `MatrixFromList` template parameters

### Phase 5: Supporting Files

10. **Update forward declarations in**:
    - [`vector.h`](../include/trackingLib/math/linalg/vector.h)
    - [`covariance_matrix_factored.h`](../include/trackingLib/math/linalg/covariance_matrix_factored.h)
    - [`modified_gram_schmidt.h`](../include/trackingLib/math/linalg/modified_gram_schmidt.h)

11. **Update [`contracts/matrix_intf.h`](../include/trackingLib/math/linalg/contracts/matrix_intf.h)**
    - Update template parameters in concepts and interface definitions

### Phase 6: Testing

12. **Update test files**:
    - Update all template instantiations in test code
    - Verify all 215 tests still pass
    - No functional changes expected, only naming

13. **Verify compilation**:
    - Build with GCC and Clang
    - Run all tests
    - Generate Doxygen documentation and verify no warnings

---

## Naming Convention Summary

### Template Parameters

| Context | Old Name | New Name | Meaning |
|---------|----------|----------|---------|
| Primary matrix | `Rows_` | `Dim0_` | First logical dimension (rows) |
| Primary matrix | `Cols_` | `Dim1_` | Second logical dimension (columns) |
| Secondary matrix (B) | `Rows2_` | `Dim0B_` | First dimension of matrix B |
| Secondary matrix (B) | `Cols2_` | `Dim1B_` | Second dimension of matrix B |
| Tertiary matrix (C) | `Rows3_` | `Dim0C_` | First dimension of matrix C |
| Tertiary matrix (C) | `Cols3_` | `Dim1C_` | Second dimension of matrix C |

### Internal Constants (Aliases)

| Constant | Value | Meaning |
|----------|-------|---------|
| `Dim0` | `Dim0_` | First logical dimension |
| `Dim1` | `Dim1_` | Second logical dimension |
| `Rows` | `Dim0_` | Number of rows (alias for user convenience) |
| `Cols` | `Dim1_` | Number of columns (alias for user convenience) |
| `RowsInMem` | `IsRowMajor_ ? Dim0_ : Dim1_` | Physical rows in memory |
| `ColsInMem` | `IsRowMajor_ ? Dim1_ : Dim0_` | Physical columns in memory |

---

## Benefits of This Refactoring

### 1. Complete Semantic Clarity
- Template parameters now accurately reflect their dimension-agnostic nature
- **No aliases** means no confusion about what "Rows" or "Cols" mean
- Consistent with the internal implementation logic
- Forces users to think correctly about dimensions vs. memory layout

### 2. Improved Documentation
- Documentation can clearly state: "Dim0_ is the first logical dimension"
- No need for conditional explanations or caveats
- Easier for users to understand the template parameters
- No confusion between template parameters and internal constants

### 3. Better Code Maintainability
- Future developers will immediately understand the dimension-agnostic design
- Reduces cognitive load when reading template-heavy code
- Makes the relationship between parameters and memory layout explicit
- Single naming convention eliminates parallel terminology

### 4. Consistency with Design
- Aligns naming with the actual implementation approach
- Makes the transpose operation more intuitive: swap Dim0 ↔ Dim1, flip IsRowMajor
- Clarifies that the Matrix class treats dimensions abstractly

### 5. Extensibility
- Clear naming convention for multi-matrix operations (Dim0B_, Dim1B_, etc.)
- Easier to add new operations involving multiple matrices
- Reduces naming conflicts and confusion

---

## Risks and Mitigation

### Risk 1: Breaking Changes for External Users
**Impact**: External code using the Matrix class will not compile after this change.

**Mitigation**:
- This is a header-only library, so no ABI compatibility issues
- Document the breaking change in release notes
- Provide migration guide showing the simple rename
- Consider this a major version bump (0.3.0 → 0.4.0 or 1.0.0)

### Risk 2: Large Scope of Changes
**Impact**: Many files need to be updated, increasing risk of errors.

**Mitigation**:
- Use systematic search-and-replace with careful review
- Update and test in phases (Core → Views → Specialized → Tests)
- Run full test suite after each phase
- Use compiler errors to catch any missed updates

### Risk 3: Documentation Inconsistency
**Impact**: Some documentation might still refer to old names.

**Mitigation**:
- Comprehensive grep for "Rows_", "Cols_", "Rows2_", "Cols2_", "IsRowMajor2_" in all files
- Update all Doxygen comments
- Regenerate documentation and review for consistency
- Update memory bank files to reflect the change

### Risk 4: Test Code Complexity
**Impact**: Test code may be harder to update due to many instantiations.

**Mitigation**:
- Tests are lower priority - update after core implementation
- Use compiler errors as a guide
- Tests should pass without functional changes (only naming)

---

## Alternative Approaches Considered

### Alternative 1: Keep Current Naming
**Rationale**: Avoid breaking changes, users are familiar with Rows/Cols.

**Rejected Because**:
- Perpetuates semantic confusion
- Documentation remains ambiguous
- Doesn't align with implementation reality
- Technical debt will accumulate

### Alternative 2: Use Height/Width Instead
**Example**: `Height_`, `Width_` instead of `Dim0_`, `Dim1_`

**Rejected Because**:
- Still implies semantic meaning (height = rows, width = columns)
- Doesn't solve the fundamental problem
- Less clear for mathematical contexts

### Alternative 3: Use Dim1/Dim2 (1-indexed)
**Example**: `Dim1_`, `Dim2_` instead of `Dim0_`, `Dim1_`

**Rejected Because**:
- Inconsistent with C++ 0-based indexing convention
- Could cause confusion with array indexing
- `Dim0_` clearly indicates "first dimension"

### Alternative 4: Add Semantic Aliases Only
**Example**: Keep `Rows_`, `Cols_` but add `Dim0`, `Dim1` aliases internally.

**Rejected Because**:
- Doesn't fix the root problem
- Creates two parallel naming systems
- Documentation still ambiguous
- Doesn't improve clarity for users

---

## Recommendation

**Proceed with the refactoring** using the proposed `Dim0_`/`Dim1_` naming convention with `Dim0B_`/`Dim1B_` for secondary matrices.

### Justification

1. **Correctness**: The current naming is semantically incorrect for column-major matrices.

2. **Clarity**: The new naming accurately reflects the dimension-agnostic design.

3. **Maintainability**: Future code will be easier to understand and maintain.

4. **Consistency**: Aligns naming with implementation reality.

5. **Extensibility**: Clear convention for multi-matrix operations.

### Implementation Priority

**HIGH PRIORITY** - This is a foundational issue that affects code clarity and correctness. While it's a breaking change, it's better to fix it now before the library is more widely adopted.

### Suggested Timeline

1. **Phase 1-2** (Core + Views): 1-2 days
2. **Phase 3-4** (Specialized + Conversions): 1 day
3. **Phase 5** (Supporting files): 0.5 days
4. **Phase 6** (Testing + Documentation): 1 day
5. **Total**: 3.5-4.5 days of focused work

---

## Additional Considerations

### Memory Bank Update

After completing this refactoring, update the memory bank files:
- [`architecture.md`](../.kilocode/rules/memory-bank/architecture.md): Update Matrix class description
- [`context.md`](../.kilocode/rules/memory-bank/context.md): Document this refactoring as completed
- [`tech.md`](../.kilocode/rules/memory-bank/tech.md): Update template parameter naming conventions

### Documentation Style

Ensure all updated Doxygen comments follow the established style:
```cpp
/// \tparam Dim0_ First logical dimension (number of rows in the matrix)
/// \tparam Dim1_ Second logical dimension (number of columns in the matrix)
/// \tparam IsRowMajor_ Storage layout: true for row-major, false for column-major
```

### Example Usage Update

Update all code examples in documentation:
```cpp
// Old
Matrix<float32, 3, 2, true> mat;  // 3 rows, 2 columns, row-major

// New (same, but clearer semantics)
Matrix<float32, 3, 2, true> mat;  // Dim0=3 (rows), Dim1=2 (cols), row-major
```

---

## Conclusion

The proposed refactoring from `Rows_`/`Cols_` to `Dim0_`/`Dim1_` is **necessary and beneficial**. It corrects a semantic inconsistency, improves code clarity, and aligns the naming with the actual implementation design. While it's a breaking change, the benefits far outweigh the costs, especially at this stage of the library's maturity.

The systematic approach outlined in this plan minimizes risks and ensures a smooth transition. All 215 tests should continue to pass after the refactoring, as this is purely a naming change with no functional modifications.

**Recommendation: Proceed with implementation.**
