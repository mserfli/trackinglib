#### 1.1 Conversion Functions Testing (COMPLETED 2026-01-02)
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
- **Implemented:** 32 tests (26 passing, 6 failing due to column-major storage issues in conversion functions)
- Added to `tests/CMakeLists.txt` and integrated into build system
