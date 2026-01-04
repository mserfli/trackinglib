# Math Layer Documentation Plan

## Executive Summary

This plan addresses the need to analyze and improve Doxygen documentation for the math layer of trackinglib. Recent refactoring efforts (print methods removal, cyclic dependency resolution, motion layer initialization) have resulted in new files being added. This analysis will ensure comprehensive documentation across all math layer components.

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

#### 1.1 Matrix Base Class (COMPLETED 2026-01-02)
**File:** [`matrix.h`](include/trackingLib/math/linalg/matrix.h)
- Enhanced comprehensive class-level documentation
- Documented all public methods with `\brief`, `\tparam`, `\param[in]`, `\return` (backslash style)
- Documented template parameters and their constraints
- Added usage examples for common Matrix operations
- Documented tl::expected error handling patterns throughout
- Added notes about row-major vs column-major storage layout
- Doxygen generates without warnings for Matrix class

#### 1.2 Specialized Matrix Classes (COMPLETED 2026-01-02)
**Files:** [`square_matrix.h`](include/trackingLib/math/linalg/square_matrix.h), [`triangular_matrix.h`](include/trackingLib/math/linalg/triangular_matrix.h), [`diagonal_matrix.h`](include/trackingLib/math/linalg/diagonal_matrix.h)
- Added comprehensive class-level documentation for all three classes
- Documented specialized operations (determinant, trace, inverse, etc.)
- Added mathematical notation and performance characteristics
- Documented decomposition algorithms (QR, LLT, LDLT, UDUT) with references to academic literature
- Added usage examples and numerical stability notes
- Doxygen generates without warnings for all three classes

#### 1.3 Vector Class (COMPLETED 2026-01-02)
**File:** [`vector.h`](include/trackingLib/math/linalg/vector.h)
- Added comprehensive class-level documentation
- Documented all public methods with \brief, \tparam, \param[in], \return
- Added mathematical notation for vector operations (dot product, norm, normalize)
- Documented relationship to Matrix class
- Doxygen generates without warnings for Vector class

### Phase 2: High Priority - Recently Added Files (Estimated: 2-3 sessions)

#### 2.1 Matrix I/O System (COMPLETED 2026-01-02)
**File:** [`matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h)
- Added comprehensive file-level documentation explaining the design and purpose
- Documented the template-based operator<< design with SFINAE-based type detection
- Documented usage with different stream types (cout, files, stringstream)
- Added detailed examples for both general matrices and DiagonalMatrix specialization
- Documented formatting behavior for different matrix types and value types
- Documented is_matrix_like trait and its helper variable
- Doxygen generates without warnings for matrix_io.h

#### 2.2 Conversion System (COMPLETED 2026-01-02)
**Files:** All files in [`conversions/`](include/trackingLib/math/linalg/conversions/) directory
- Added comprehensive Doxygen documentation for all conversion functions in each .hpp file
- Documented the `<target>From<source>` naming convention and function overloading strategy
- Added usage examples and cross-references between related conversions
- Documented error handling (runtime_error for dimension mismatches, assertions for list sizes)
- Doxygen generates without warnings for all conversion functions
- Each function has \brief, \tparam, \param[in], \return, and \see tags

#### 2.3 Decomposition Implementations (COMPLETED 2026-01-02)
**File:** [`square_matrix_decompositions.hpp`](include/trackingLib/math/linalg/square_matrix_decompositions.hpp)
- Added comprehensive Doxygen documentation for all four decomposition algorithms (Householder QR, LLT, LDLT, UDUT)
- Documented mathematical background, preconditions, numerical stability, and complexity analysis
- Added academic references (Grewal & Andrews, Bierman, Thornton, etc.)
- Documented error handling and return types
- Doxygen generates without warnings for decomposition functions

### Phase 3: Medium Priority - View Classes (Estimated: 2 sessions) (COMPLETED 2026-01-02)

#### 3.1 Matrix Views
**Files:** [`matrix_view.h`](include/trackingLib/math/linalg/matrix_view.h), [`matrix_row_view.h`](include/trackingLib/math/linalg/matrix_row_view.h), [`matrix_column_view.h`](include/trackingLib/math/linalg/matrix_column_view.h)
- Added comprehensive class-level documentation for MatrixRowView and MatrixColumnView
- Documented non-owning view concept for single row/column views
- Documented lifetime and safety considerations
- Documented all supported operations (matrix multiplication, dot products, element access)
- Added usage examples and performance notes (zero-copy operations)
- Documented all public methods with \brief, \tparam, \param[in], \return
- Added cross-references between view classes
- Doxygen generates without warnings for both view classes

### Phase 4: Medium Priority - Covariance Classes (COMPLETED 2026-01-02)

#### 4.1 Covariance Matrices
**Files:** [`covariance_matrix_full.h`](include/trackingLib/math/linalg/covariance_matrix_full.h), [`covariance_matrix_factored.h`](include/trackingLib/math/linalg/covariance_matrix_factored.h)
- Added comprehensive class-level documentation for CovarianceMatrixFull explaining covariance matrix properties (symmetry, positive semi-definiteness)
- Added comprehensive class-level documentation for CovarianceMatrixFactored explaining UDU factorization concept, benefits, and algorithms
- Documented Thornton update algorithm with mathematical notation and Kalman filtering context
- Documented Agee-Turner rank-1 update with numerical stability notes
- Added references to academic publications (Thornton, Bierman, D'Souza)
- Documented numerical stability advantages of factored representation
- Enhanced method documentation with complexity analysis and usage notes
- Doxygen generates without warnings for both covariance matrix classes

### Phase 5: Medium Priority - Specialized Operations (COMPLETED 2026-01-02)

#### 5.1 Rank-1 Updates
**File:** [`rank1_update.h`](include/trackingLib/math/linalg/rank1_update.h)
- Added comprehensive class-level documentation for Rank1Update class
- Documented rank-1 update algorithms for UDU and LDL factorizations
- Added mathematical background and numerical stability considerations
- Referenced academic sources (Gill et al., Thornton)
- Documented Kalman filtering applications and usage contexts
- Doxygen generates without warnings

#### 5.2 Modified Gram-Schmidt
**File:** [`modified_gram_schmidt.h`](include/trackingLib/math/linalg/modified_gram_schmidt.h)
- Added comprehensive class-level documentation for ModifiedGramSchmidt class
- Documented MGS algorithm for UDU factorization in Kalman filtering context
- Added mathematical background and numerical stability properties
- Referenced academic sources (Thornton, Grewal & Andrews)
- Documented usage in covariance matrix propagation
- Doxygen generates without warnings

### Phase 6: Low Priority - Utility Files (COMPLETED 2026-01-02)

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

## Phase 2.2 Conversion System Documentation Strategy

### Question: Should we add conversion function declarations to conversions.h?

**Answer:** No, we should NOT add conversion function declarations to [`conversions.h`](include/trackingLib/math/linalg/conversions/conversions.h) because it would reintroduce circular dependencies.

### Detailed Analysis:

#### Current Design (Circular-Dependency-Free):
- [`conversions.h`](include/trackingLib/math/linalg/conversions/conversions.h) contains ONLY forward declarations
- Each conversion function is implemented in separate `.hpp` files (e.g., `matrix_conversions.hpp`, `diagonal_conversions.hpp`)
- These implementation files include the necessary matrix class headers
- No circular dependencies exist because the master header doesn't include implementation details

#### Why Adding Declarations Would Cause Problems:
1. **Circular Dependency Risk**: If we add function declarations with full signatures, we'd need to include matrix class headers, which could create circular dependencies
2. **Header Bloat**: The master header would become much larger and more complex
3. **Compilation Time**: Including all matrix headers in the master file would increase compilation times
4. **Defeats Purpose**: The current design successfully eliminated circular dependencies - adding declarations would undermine this achievement

#### Recommended Documentation Strategy:
1. **Document conversions.h**: Add comprehensive file-level documentation explaining the architecture (already done)
2. **Document each conversion function**: Add Doxygen documentation to each function in its respective implementation file
3. **Add cross-references**: Use `\see` commands to link between related conversions
4. **Document in implementation files**: Each conversion function should have full Doxygen documentation in its `.hpp` file

#### Example of Proper Documentation Location:
```cpp
// In diagonal_conversions.hpp:
/// \brief Converts a SquareMatrix to a DiagonalMatrix
///
/// Extracts the diagonal elements from a square matrix to create a diagonal matrix.
/// This conversion preserves only the diagonal elements and discards off-diagonal values.
///
/// \tparam ValueType_ The value type of matrix elements
/// \tparam Size_ The size of the square matrix
/// \tparam IsRowMajor_ The storage layout of the source matrix
/// \param mat The source square matrix
/// \return DiagonalMatrix containing the diagonal elements of the input matrix
/// \see DiagonalFromList() for creating diagonal matrices from initializer lists
/// \see SquareFromDiagonal() for the reverse conversion
inline auto DiagonalFromSquare(const SquareMatrix<ValueType_, Size_, IsRowMajor_>& mat) -> DiagonalMatrix<ValueType_, Size_>
{
    // implementation...
}
```

#### Benefits of Current Approach:
- **Maintains circular-dependency-free design**: No risk of reintroducing the problems we just solved
- **Better organization**: Documentation is co-located with implementation
- **Easier maintenance**: Changes to conversion functions don't require updating a central header
- **Faster compilation**: Minimal includes in the master header
- **Clear separation**: Architecture documentation in master header, function documentation with implementations

### Implementation Plan for Phase 2.2:

1. **Step 1**: Document [`conversions.h`](include/trackingLib/math/linalg/conversions/conversions.h) with architecture overview (COMPLETED)
2. **Step 2**: Document each conversion function in its respective `.hpp` file:
   - `matrix_conversions.hpp`: `MatrixFromList()`, `MatrixFromVector()`
   - `diagonal_conversions.hpp`: `DiagonalFromSquare()`, `DiagonalFromList()` (both overloads)
   - `square_conversions.hpp`: `SquareFromList()`, `SquareFromDiagonal()`
   - `vector_conversions.hpp`: `VectorFromList()`, `VectorFromMatrixColumnView()`
   - `triangular_conversions.hpp`: `TriangularFromSquare()`, `TriangularFromList()`
   - `covariance_matrix_conversions.hpp`: `CovarianceMatrixFromList()` (both full and factored)
3. **Step 3**: Add usage examples and cross-references
4. **Step 4**: Generate Doxygen and verify all conversions appear in documentation

This approach maintains the clean architecture while providing comprehensive documentation coverage.

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