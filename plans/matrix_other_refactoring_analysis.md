# Matrix Classes Refactoring and Coverage Plan

## Status Summary

This plan tracks remaining work for matrix class refactoring and test coverage improvements. Several items from the original analysis have been completed.

### ✅ Completed Items
- **MatrixView Critical Bug**: Fixed - the owning version with dangling reference has been removed
- **MatrixView Tests**: Created [`tests/math/test_matrix_view.cpp`](tests/math/test_matrix_view.cpp) with comprehensive coverage
- **Point2d Tests**: Populated [`tests/math/test_point2d.cpp`](tests/math/test_point2d.cpp) with full coverage
- **Point3d Tests**: Created [`tests/math/test_point3d.cpp`](tests/math/test_point3d.cpp) with full coverage
- **Vector::FromMatrixColumnView**: Test exists in [`tests/math/test_vector.cpp`](tests/math/test_vector.cpp:20)
- **SquareMatrix::decomposeUDUT**: Tests exist in [`tests/math/test_square_matrix.cpp`](tests/math/test_square_matrix.cpp:163)
- **SquareMatrix::symmetrize**: Test exists in [`tests/math/test_square_matrix.cpp`](tests/math/test_square_matrix.cpp:119)

---

## 1. Remaining Test Coverage Gaps

### SquareMatrix Constructor from DiagonalMatrix
**Status**: ✅ Tested

**Issue**: [`SquareMatrix`](include/trackingLib/math/linalg/square_matrix.h) has a constructor that accepts [`DiagonalMatrix`](include/trackingLib/math/linalg/diagonal_matrix.h).

**Status**: Test added in [`tests/math/test_square_matrix.cpp`](tests/math/test_square_matrix.cpp:146) that verifies:
- Diagonal elements are correctly copied
- Off-diagonal elements are zero

---

## 2. Code Quality Issues

### 2.1 Print Methods Using std::cout
**Status**: ⚠️ Moved to separate plan

**Issue**: Multiple classes have `print()` methods that are not idiomatic C++.

**See**: [`plans/print_methods_refactoring.md`](plans/print_methods_refactoring.md) for comprehensive analysis including:
- Template-based `operator<<` solution with zero code duplication
- Unified formatting for all matrix types
- Migration strategy from `print()` to `operator<<`
- Advanced features (custom formatting, JSON/CSV output)
- Removes circular dependencies caused by print methods

### 2.2 OpenMP Dependency
**Status**: ⚠️ Moved to separate plan

**Issue**: Inconsistent OpenMP usage across matrix multiplication operations.

**See**: [`plans/openmp_parallelization_analysis.md`](plans/openmp_parallelization_analysis.md) for comprehensive analysis and strategy.

---

## 3. Cyclic Dependencies

**Status**: ⚠️ Moved to separate plan

**Issue**: Multiple circular dependencies between `SquareMatrix`, `DiagonalMatrix`, and `TriangularMatrix`.

**See**: [`plans/cyclic_dependencies_analysis.md`](plans/cyclic_dependencies_analysis.md) for comprehensive analysis including:
- Detailed dependency graph with Mermaid diagrams
- Analysis of all three cycles
- Impact on compilation times and maintainability
- Multiple resolution strategies with pros/cons
- Phased implementation plan
- Before/after dependency graphs

---

## 4. Execution Priority

### High Priority (Blocking Issues)
1. ✅ ~~Fix MatrixView critical bug~~ - **COMPLETED**
2. ✅ ~~Add MatrixView tests~~ - **COMPLETED**

### Medium Priority (Test Coverage)
3. ✅ ~~Add Point2d/Point3d tests~~ - **COMPLETED**
4. ✅ ~~Add Vector::FromMatrixColumnView test~~ - **COMPLETED**
5. ✅ ~~Add SquareMatrix::decomposeUDUT test~~ - **COMPLETED**
6. ✅ ~~Add SquareMatrix::symmetrize test~~ - **COMPLETED**
7. ✅ ~~Add SquareMatrix(DiagonalMatrix) constructor test~~ - **COMPLETED**

### Low Priority (Code Quality)
8. ⚠️ **Refactor print() methods** - See separate plan: [`print_methods_refactoring.md`](plans/print_methods_refactoring.md)
9. ⚠️ **Break cyclic dependencies** - See separate plan: [`cyclic_dependencies_analysis.md`](plans/cyclic_dependencies_analysis.md)
10. ⚠️ **OpenMP parallelization** - See separate plan: [`openmp_parallelization_analysis.md`](plans/openmp_parallelization_analysis.md)

---

## Next Steps

### Future Improvements
1. **See Separate Plans**:
   - Print methods: [`print_methods_refactoring.md`](plans/print_methods_refactoring.md)
   - Cyclic dependencies: [`cyclic_dependencies_analysis.md`](plans/cyclic_dependencies_analysis.md)
   - OpenMP parallelization: [`openmp_parallelization_analysis.md`](plans/openmp_parallelization_analysis.md)

---

## Notes

- All critical bugs have been resolved
- Test coverage is significantly improved
- Remaining work is primarily code quality and architectural improvements
- No blocking issues remain for library functionality
