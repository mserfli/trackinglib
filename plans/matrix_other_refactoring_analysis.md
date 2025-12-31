# Matrix Classes Refactoring and Coverage Plan

## 1. Matrix Views (`MatrixColumnView`, `MatrixRowView`, `MatrixView`)

### Findings
*   **`MatrixColumnView` / `MatrixRowView`**:
    *   **Coverage**: Good. Arithmetic operations are tested.
    *   **Issues**: `print()` method writes directly to `std::cout`.
*   **`MatrixView`**:
    *   **CRITICAL BUG**: `MatrixView<FloatType, Rows, Cols, false>` (the owning version) initializes a reference member `_view` with a temporary object in the constructor initializer list. This results in a dangling reference.
    *   **Coverage**: **ZERO**. No test file exists.
    *   **Issues**: Uses `#pragma omp parallel for` which introduces an OpenMP dependency.

### Plan
1.  **Fix Critical Bug**: Refactor `MatrixView<..., false>` to store the view by value or reimplement the accessors to avoid the dangling reference.
2.  **Add Tests**: Create `tests/math/test_matrix_view.cpp` covering:
    *   Construction (view on existing matrix, owning view).
    *   Arithmetic operations (`+`, `-`, `*`, `/`).
    *   Interaction with `Matrix`.
3.  **Refactoring**:
    *   Consider removing `print()` or replacing with `operator<<`.

## 2. Vector and Points (`Vector`, `Point2d`, `Point3d`)

### Findings
*   **`Vector`**:
    *   **Coverage**: Mostly good.
    *   **Missing**: `FromMatrixColumnView` is not tested.
*   **`Point2d`**:
    *   **Coverage**: **ZERO**. `tests/math/test_point2d.cpp` exists but is empty.
*   **`Point3d`**:
    *   **Coverage**: **ZERO**. No test file exists.

### Plan
1.  **Add Tests**:
    *   Populate `tests/math/test_point2d.cpp` (constructors, accessors `x/y`, inheritance from Vector).
    *   Create `tests/math/test_point3d.cpp` (constructors, accessors `x/y/z`, inheritance from Vector).
    *   Add test for `Vector::FromMatrixColumnView` in `tests/math/test_vector.cpp`.

## 3. Square Matrix (`SquareMatrix`)

### Findings
*   **Coverage**: Good for most complex algorithms (`householderQR`, `decomposeLLT`, `decomposeLDLT`, `inverse`).
*   **Missing**:
    *   `decomposeUDUT` (Critical for UDU factorization feature).
    *   `symmetrize`.
    *   Constructor from `DiagonalMatrix`.

### Plan
1.  **Add Tests**:
    *   Add `decomposeUDUT` test case (verify reconstruction $P = U D U^T$).
    *   Add `symmetrize` test case.
    *   Add `SquareMatrix(DiagonalMatrix)` test case.

## 4. Diagonal and Triangular Matrices

### Findings
*   **`DiagonalMatrix`**:
    *   **Coverage**: Good.
    *   **Issues**: `print()` uses `std::cout`.
*   **`TriangularMatrix`**:
    *   **Coverage**: Good.
    *   **Issues**: Uses `#pragma omp parallel for`.

### Plan
1.  **Refactoring**:
    *   Review OpenMP usage.
    *   Consider `print()` refactoring.

## 5. Cyclic Dependencies

### Findings
*   **`SquareMatrix` <-> `DiagonalMatrix`**:
    *   `SquareMatrix` uses `DiagonalMatrix` (Identity, setIdentity).
    *   `DiagonalMatrix` uses `SquareMatrix` (FromMatrix, print).
    *   Both include each other's `.hpp` files.
*   **`SquareMatrix` <-> `TriangularMatrix`**:
    *   `TriangularMatrix` inherits `SquareMatrix`.
    *   `SquareMatrix` uses `TriangularMatrix` (decompositions).
    *   Both include each other's `.hpp` files.

### Plan
1.  **Break `SquareMatrix` <-> `DiagonalMatrix` Cycle**:
    *   Refactor `DiagonalMatrix::print()` to remove dependency on `SquareMatrix`.
    *   Move `DiagonalMatrix::FromMatrix` to a new header `diagonal_matrix_utils.hpp` (or similar) to remove the dependency from the core class.
    *   Remove `#include "square_matrix.hpp"` from `diagonal_matrix.hpp`.
2.  **Break `SquareMatrix` <-> `TriangularMatrix` Cycle**:
    *   Move decomposition methods (`householderQR`, `decomposeLLT`, `decomposeLDLT`, `decomposeUDUT`) from `square_matrix.hpp` to a new header `square_matrix_decompositions.hpp`.
    *   Remove `#include "triangular_matrix.hpp"` from `square_matrix.hpp`.

## Execution Order
1.  **Fix `MatrixView`**: This is a blocker/critical bug.
2.  **Add `MatrixView` Tests**: Verify fix.
3.  **Add Point/Vector Tests**: Low hanging fruit, high value for completeness.
4.  **Add `SquareMatrix` Tests**: Ensure core algorithms are fully covered.
5.  **Refactor Cycles**: Perform the structural changes to break cyclic dependencies.
