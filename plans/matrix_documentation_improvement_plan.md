# Matrix Internal Constants Refactoring and Documentation Improvement Plan

## Executive Summary

**Issue**: The Matrix class has confusing internal constant names that obscure the distinction between layout-dependent template parameters and logical dimensions.

**Key Insight**: `RowsInMem` and `ColsInMem` are actually **logical dimensions** (what the user sees), not physical memory dimensions. The naming is backwards!

**Solution**: 
1. Rename internal constants: `RowsInMem` → `LogicalRows`, `ColsInMem` → `LogicalCols`
2. Improve documentation to explain layout-dependent template parameters
3. Keep `Rows` and `Cols` as layout-dependent aliases

---

## Problem Analysis

### The Actual Implementation

```cpp
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_ = true>
class Matrix
{
  static constexpr auto Rows       = Rows_;
  static constexpr auto Cols       = Cols_;
  static constexpr auto RowsInMem  = IsRowMajor_ ? Rows_ : Cols_;  // Actually LOGICAL rows!
  static constexpr auto ColsInMem  = IsRowMajor_ ? Cols_ : Rows_;  // Actually LOGICAL cols!
  // ...
};
```

### The Confusion

The names `RowsInMem` and `ColsInMem` suggest "physical rows/columns in memory", but they actually represent:
- **Logical rows** (what the user sees as rows)
- **Logical columns** (what the user sees as columns)

### The Truth

- **Template parameters** (`Rows_`, `Cols_`): Layout-dependent
  - Row-major: `Rows_` = rows, `Cols_` = columns
  - Column-major: `Rows_` = columns, `Cols_` = rows (swapped!)

- **Constants** (`Rows`, `Cols`): Just aliases for template parameters (layout-dependent)

- **Constants** (`RowsInMem`, `ColsInMem`): **Logical dimensions** (always correct from user perspective)
  - Row-major: `RowsInMem = Rows_`, `ColsInMem = Cols_`
  - Column-major: `RowsInMem = Cols_`, `ColsInMem = Rows_` (swapped back!)

### Example

```cpp
Matrix<float, 3, 2, true> mat1;   // Row-major
// Rows_ = 3, Cols_ = 2
// Rows = 3, Cols = 2
// RowsInMem = 3, ColsInMem = 2  (logical: 3 rows × 2 cols) ✓

Matrix<float, 2, 3, false> mat2;  // Column-major
// Rows_ = 2, Cols_ = 3
// Rows = 2, Cols = 3
// RowsInMem = 3, ColsInMem = 2  (logical: 3 rows × 2 cols) ✓
```

Both matrices are logically 3×2, but template parameters are swapped for column-major!

---

## Proposed Solution

### Phase 1: Rename Internal Constants

**File**: [`include/trackingLib/math/linalg/matrix.h`](../include/trackingLib/math/linalg/matrix.h)

**Current** (lines ~94-100):
```cpp
static constexpr auto Rows       = Rows_;
static constexpr auto Cols       = Cols_;
static constexpr auto RowsInMem  = IsRowMajor_ ? Rows_ : Cols_;
static constexpr auto ColsInMem  = IsRowMajor_ ? Cols_ : Rows_;
static constexpr auto IsRowMajor = IsRowMajor_;
```

**Proposed**:
```cpp
static constexpr auto Rows        = Rows_;                        ///< First dimension (layout-dependent, = Rows_ template parameter)
static constexpr auto Cols        = Cols_;                        ///< Second dimension (layout-dependent, = Cols_ template parameter)
static constexpr auto LogicalRows = IsRowMajor_ ? Rows_ : Cols_;  ///< Logical number of rows (user-facing, always correct)
static constexpr auto LogicalCols = IsRowMajor_ ? Cols_ : Rows_;  ///< Logical number of columns (user-facing, always correct)
static constexpr auto IsRowMajor  = IsRowMajor_;                  ///< Memory layout: true=row-major, false=column-major
```

**Rationale**:
- `Rows` and `Cols` remain as aliases for template parameters (layout-dependent)
- `LogicalRows` and `LogicalCols` replace `RowsInMem` and `ColsInMem` (much clearer!)
- The old names were misleading - they're not "in memory", they're logical dimensions!

### Phase 2: Update All Usages

**Search and replace across all files**:
- `RowsInMem` → `LogicalRows`
- `ColsInMem` → `LogicalCols`

**Files to update** (estimated):
1. [`matrix.h`](../include/trackingLib/math/linalg/matrix.h)
2. [`matrix.hpp`](../include/trackingLib/math/linalg/matrix.hpp)
3. [`matrix_view.h`](../include/trackingLib/math/linalg/matrix_view.h)
4. [`matrix_view.hpp`](../include/trackingLib/math/linalg/matrix_view.hpp)
5. [`matrix_row_view.h`](../include/trackingLib/math/linalg/matrix_row_view.h)
6. [`matrix_row_view.hpp`](../include/trackingLib/math/linalg/matrix_row_view.hpp)
7. [`matrix_column_view.h`](../include/trackingLib/math/linalg/matrix_column_view.h)
8. [`matrix_column_view.hpp`](../include/trackingLib/math/linalg/matrix_column_view.hpp)
9. [`conversions/matrix_conversions.hpp`](../include/trackingLib/math/linalg/conversions/matrix_conversions.hpp)
10. [`conversions/square_conversions.hpp`](../include/trackingLib/math/linalg/conversions/square_conversions.hpp)
11. Test files

**Estimated occurrences**: ~30-50 locations

### Phase 3: Update Template Parameter Documentation

**File**: [`include/trackingLib/math/linalg/matrix.h`](../include/trackingLib/math/linalg/matrix.h)

**Current** (lines ~39-42):
```cpp
/// \tparam ValueType_ The data type of matrix elements
/// \tparam Rows_ The number of rows in the matrix
/// \tparam Cols_ The number of columns in the matrix
/// \tparam IsRowMajor_ Storage layout: true for row-major, false for column-major
```

**Proposed**:
```cpp
/// \tparam ValueType_ The data type of matrix elements (typically arithmetic types like float32, float64, int)
/// \tparam Rows_ First dimension in the specified memory layout (must be positive, > 0)
///                - For row-major (IsRowMajor_=true): number of rows
///                - For column-major (IsRowMajor_=false): number of columns (swapped!)
/// \tparam Cols_ Second dimension in the specified memory layout (must be positive, > 0)
///                - For row-major (IsRowMajor_=true): number of columns
///                - For column-major (IsRowMajor_=false): number of rows (swapped!)
/// \tparam IsRowMajor_ Storage layout: true for row-major order, false for column-major order
```

### Phase 4: Add Comprehensive Class Documentation

**Add after template parameter documentation**:

```cpp
/// \section layout_semantics Memory Layout and Template Parameter Semantics
///
/// The Matrix class uses layout-dependent template parameters where Rows_ and Cols_ represent
/// dimensions in the specified memory layout order, not necessarily logical rows and columns.
///
/// \subsection row_major Row-Major Layout (IsRowMajor_ = true)
/// Elements are stored row by row in memory. Template parameters match logical dimensions:
/// - Rows_ = number of logical rows
/// - Cols_ = number of logical columns
/// - Memory order: [row0_col0, row0_col1, ..., row1_col0, row1_col1, ...]
///
/// Example:
/// \code{.cpp}
/// Matrix<float, 3, 2, true> mat;  // 3 rows × 2 columns, row-major
/// // Template: <3, 2>  →  Logical: 3×2  ✓ (matches)
/// // mat.Rows = 3, mat.Cols = 2 (layout-dependent)
/// // mat.LogicalRows = 3, mat.LogicalCols = 2 (always correct)
/// \endcode
///
/// \subsection col_major Column-Major Layout (IsRowMajor_ = false)
/// Elements are stored column by column in memory. Template parameters are SWAPPED:
/// - Rows_ = number of logical COLUMNS (first dimension in memory)
/// - Cols_ = number of logical ROWS (second dimension in memory)
/// - Memory order: [col0_row0, col0_row1, ..., col1_row0, col1_row1, ...]
///
/// Example:
/// \code{.cpp}
/// Matrix<float, 2, 3, false> mat;  // 3 rows × 2 columns, column-major
/// // Template: <2, 3>  →  Logical: 3×2  ✗ (swapped!)
/// // mat.Rows = 2, mat.Cols = 3 (layout-dependent, WRONG for logical view!)
/// // mat.LogicalRows = 3, mat.LogicalCols = 2 (always correct)
/// \endcode
///
/// \subsection transpose_semantics Transpose Semantics
/// The transpose operation swaps Rows_ ↔ Cols_ and flips IsRowMajor_. This creates a
/// zero-copy view of the same data with swapped logical dimensions:
///
/// \code{.cpp}
/// Matrix<float, 3, 2, true> mat;           // 3×2 row-major
/// auto matT = mat.transpose();             // Returns Matrix<float, 2, 3, false>&
/// // matT is a 2×3 column-major view of the same data (zero-copy)
/// \endcode
///
/// \subsection user_interface User Interface
/// Despite the layout-dependent template parameters, the user interface is always logical:
/// - at_unsafe(row, col) and operator()(row, col) use logical row/column indices
/// - LogicalRows and LogicalCols constants always return correct dimensions
/// - All operations work transparently across different layouts
///
/// \warning When instantiating column-major matrices, remember that Rows_ and Cols_ are swapped!
///          Always use LogicalRows and LogicalCols to get the correct dimensions.
```

### Phase 5: Add Usage Examples

```cpp
/// \section examples Usage Examples
///
/// \subsection ex_basic Basic Matrix Creation
/// \code{.cpp}
/// // Create a 3×2 row-major matrix
/// auto mat1 = Matrix<float, 3, 2, true>::Zeros();
/// std::cout << "Template: " << mat1.Rows << "×" << mat1.Cols << std::endl;        // 3×2
/// std::cout << "Logical: " << mat1.LogicalRows << "×" << mat1.LogicalCols << std::endl;  // 3×2
///
/// // Create a 3×2 column-major matrix (note swapped template parameters!)
/// auto mat2 = Matrix<float, 2, 3, false>::Zeros();
/// std::cout << "Template: " << mat2.Rows << "×" << mat2.Cols << std::endl;        // 2×3 (WRONG!)
/// std::cout << "Logical: " << mat2.LogicalRows << "×" << mat2.LogicalCols << std::endl;  // 3×2 (correct!)
/// \endcode
///
/// \subsection ex_transpose Transpose Operations
/// \code{.cpp}
/// Matrix<float, 3, 2, true> mat;  // 3×2 row-major
/// mat.at_unsafe(0, 0) = 1.0f;
/// mat.at_unsafe(0, 1) = 2.0f;
///
/// auto matT = mat.transpose();    // 2×3 column-major view (zero-copy)
/// // matT.at_unsafe(0, 0) == 1.0f  (same element)
/// // matT.at_unsafe(1, 0) == 2.0f  (was mat(0,1))
/// \endcode
```

---

## Implementation Plan

### Phase 0: Critical Bug Fix - Logical vs Layout-Dependent Dimensions (1 day)

**Issue**: Throughout [`matrix.hpp`](../include/trackingLib/math/linalg/matrix.hpp), logical coordinates are compared against layout-dependent dimensions (`Rows`, `Cols`), causing bugs for column-major matrices.

**Scope**: **12 locations** in [`matrix.hpp`](../include/trackingLib/math/linalg/matrix.hpp) need fixing

**All other files are SAFE** - only the core Matrix class is affected.

#### Detailed Fixes

**1. `at_unsafe()` assertions** (lines 49-50, 58-59):
```cpp
// Current (WRONG):
assert((0 <= row) && (row < Rows));
assert((0 <= col) && (col < Cols));

// Fixed:
assert((0 <= row) && (row < RowsInMem));
assert((0 <= col) && (col < ColsInMem));
```

**2. `operator()` bounds checks** (lines 68-69, 83-84):
```cpp
// Current (WRONG):
if (!(row >= 0 && row < Rows))
if (!(col >= 0 && col < Cols))

// Fixed:
if (!(row >= 0 && row < RowsInMem))
if (!(col >= 0 && col < ColsInMem))
```

**3. `operator==` loop bounds** (lines 105-107):
```cpp
// Current (WRONG):
for (auto row = 0; row < Rows; ++row)
{
  for (auto col = 0; col < Cols; ++col)

// Fixed:
for (auto row = 0; row < RowsInMem; ++row)
{
  for (auto col = 0; col < ColsInMem; ++col)
```

**4. `operator+=` loop bounds** (lines 144-146, 158-160):
```cpp
// Current (WRONG):
for (auto row = 0; row < Rows; ++row)
{
  for (auto col = 0; col < Cols; ++col)

// Fixed:
for (auto row = 0; row < RowsInMem; ++row)
{
  for (auto col = 0; col < ColsInMem; ++col)
```

**5. `operator-=` loop bounds** (lines 184-186, 197-199):
```cpp
// Current (WRONG):
for (auto row = 0; row < Rows; ++row)
{
  for (auto col = 0; col < Cols; ++col)

// Fixed:
for (auto row = 0; row < RowsInMem; ++row)
{
  for (auto col = 0; col < ColsInMem; ++col)
```

**6. `operator*` (matrix multiplication) loop bounds** (lines 325-327, 329):
```cpp
// Current (WRONG):
for (auto i = 0; i < ResultMatrix::Rows; ++i)
{
  for (auto j = 0; j < ResultMatrix::Cols; ++j)
  {
    for (auto k = 0; k < Cols; ++k)

// Fixed:
for (auto i = 0; i < ResultMatrix::RowsInMem; ++i)
{
  for (auto j = 0; j < ResultMatrix::ColsInMem; ++j)
  {
    for (auto k = 0; k < ColsInMem; ++k)
```

**7. `setBlock()` runtime assertions** (lines 418-419):
```cpp
// Current (WRONG):
assert((dstRowBeg + srcRowCount <= Rows) && "copy to many rows to dst");
assert((dstColBeg + srcColCount <= Cols) && "copy to many cols to dst");

// Fixed:
assert((dstRowBeg + srcRowCount <= RowsInMem) && "copy to many rows to dst");
assert((dstColBeg + srcColCount <= ColsInMem) && "copy to many cols to dst");
```

#### Test Updates Required

**File**: [`tests/math/test_matrix.cpp`](../tests/math/test_matrix.cpp)

Add comprehensive tests for column-major matrices:

1. **Element access tests**:
   ```cpp
   TEST(GTestMatrix, at_unsafe_ColumnMajor_BoundaryConditions__Success)
   {
     Matrix<float, 2, 3, false> mat;  // Logical: 3×2 column-major
     // Test all boundary conditions
     mat.at_unsafe(0, 0) = 1.0f;  // Top-left
     mat.at_unsafe(2, 1) = 6.0f;  // Bottom-right (row=2, col=1 valid for 3×2)
     EXPECT_EQ(mat.at_unsafe(0, 0), 1.0f);
     EXPECT_EQ(mat.at_unsafe(2, 1), 6.0f);
   }
   ```

2. **Operator tests**:
   ```cpp
   TEST(GTestMatrix, operator_call_ColumnMajor__Success)
   {
     Matrix<float, 2, 3, false> mat;  // Logical: 3×2 column-major
     auto result = mat(2, 1);  // Should succeed
     EXPECT_TRUE(result.has_value());
   }
   ```

3. **Arithmetic operation tests**:
   ```cpp
   TEST(GTestMatrix, operator_plus_equals_ColumnMajor__Success)
   {
     Matrix<float, 2, 3, false> mat1;  // Logical: 3×2 column-major
     Matrix<float, 2, 3, false> mat2;
     mat1 += mat2;  // Should not assert
   }
   ```

4. **Matrix multiplication tests**:
   ```cpp
   TEST(GTestMatrix, operator_multiply_ColumnMajor__Success)
   {
     Matrix<float, 3, 2, false> mat1;  // Logical: 2×3 column-major
     Matrix<float, 2, 4, false> mat2;  // Logical: 4×2 column-major
     auto result = mat1 * mat2;  // Should work correctly
   }
   ```

5. **setBlock tests**:
   ```cpp
   TEST(GTestMatrix, setBlock_ColumnMajor__Success)
   {
     Matrix<float, 3, 4, false> dst;  // Logical: 4×3 column-major
     Matrix<float, 2, 2, false> src;  // Logical: 2×2 column-major
     dst.setBlock(2, 2, 0, 0, 0, 0, src);  // Should not assert
   }
   ```

### Phase 1: Rename Internal Constants (0.5 days)
1. Update constant definitions in [`matrix.h`](../include/trackingLib/math/linalg/matrix.h)
2. Add documentation for each constant

### Phase 2: Update All Usages (1 day)
1. Search and replace `RowsInMem` → `LogicalRows` across all files
2. Search and replace `ColsInMem` → `LogicalCols` across all files
3. Verify compilation after each file
4. Run tests to ensure no regressions

### Phase 3: Update Documentation (1 day)
1. Update template parameter documentation
2. Add comprehensive class-level documentation
3. Add usage examples
4. Update related files (views, etc.)

### Phase 4: Memory Bank Updates (0.5 days)
1. Update [`architecture.md`](../.kilocode/rules/memory-bank/architecture.md)
2. Update [`context.md`](../.kilocode/rules/memory-bank/context.md)
3. Update [`tech.md`](../.kilocode/rules/memory-bank/tech.md)

### Phase 5: Testing and Verification (0.5 days)
1. Run all tests (should pass, including new column-major tests)
2. Generate Doxygen documentation
3. Verify no warnings
4. Review for consistency

**Total Estimated Effort**: 4 days (added Phase 0 for critical bug fix)

---

## Benefits

1. **Clearer Naming**: `LogicalRows`/`LogicalCols` immediately convey their meaning
2. **Less Confusion**: Eliminates the misleading "InMem" terminology
3. **Better Documentation**: Comprehensive explanation of layout-dependent semantics
4. **Backward Compatibility**: `Rows` and `Cols` remain unchanged
5. **User-Friendly**: Always use `LogicalRows`/`LogicalCols` for correct dimensions

---

## Breaking Changes

**Impact**: Code using `RowsInMem` or `ColsInMem` must be updated

**Migration**:
- Replace `RowsInMem` with `LogicalRows`
- Replace `ColsInMem` with `LogicalCols`

**Estimated Impact**: Low (these constants are primarily used internally)

---

## Conclusion

The key insight is that `RowsInMem` and `ColsInMem` are **not** physical memory dimensions - they're logical dimensions that are always correct from the user's perspective. Renaming them to `LogicalRows` and `LogicalCols` makes this explicit and eliminates confusion.

This refactoring, combined with improved documentation, will make the Matrix class much easier to understand and use correctly.
