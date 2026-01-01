# Print Methods Refactoring Plan

## Overview

This plan addresses the refactoring of `print()` methods across all matrix classes in trackinglib to use idiomatic C++ `operator<<` with a unified, reusable implementation that avoids code duplication.

**Requirements**:
- Must be compatible with C++17 and higher
- Must support future memory layout optimizations (e.g., packed triangular matrices)
- Must avoid code duplication across matrix types

## Current State

### Files with print() Methods

| File | Class | Implementation | Issues |
|------|-------|----------------|--------|
| [`matrix.h`](include/trackingLib/math/linalg/matrix.h:196) | `Matrix` | [`matrix.hpp:78`](include/trackingLib/math/linalg/matrix.hpp:78) | Direct `std::cout`, duplicated logic |
| [`matrix_row_view.h`](include/trackingLib/math/linalg/matrix_row_view.h:64) | `MatrixRowView` | [`matrix_row_view.hpp:74`](include/trackingLib/math/linalg/matrix_row_view.hpp:74) | Direct `std::cout` |
| [`matrix_column_view.h`](include/trackingLib/math/linalg/matrix_column_view.h:60) | `MatrixColumnView` | [`matrix_column_view.hpp:67`](include/trackingLib/math/linalg/matrix_column_view.hpp:67) | Direct `std::cout` |
| [`diagonal_matrix.h`](include/trackingLib/math/linalg/diagonal_matrix.h:148) | `DiagonalMatrix` | Not yet implemented | Planned to convert to `SquareMatrix` |
| [`covariance_matrix_factored.h`](include/trackingLib/math/linalg/covariance_matrix_factored.h:127) | `CovarianceMatrixFactored` | Inline delegation | Delegates to underlying matrix |

### Problems with Current Approach

1. **Not Idiomatic C++**: Using `print()` instead of `operator<<`
2. **Hardcoded Output**: All output goes to `std::cout`, can't redirect to files or strings
3. **Code Duplication**: Similar printing logic repeated across classes
4. **Testing Difficulty**: Can't easily test output without capturing stdout
5. **Inflexibility**: Can't customize formatting per use case
6. **Dependency Issues**: `DiagonalMatrix::print()` creates circular dependency

## Design Goals

1. **Idiomatic C++**: Use `operator<<` for stream output
2. **Reusable**: Single implementation for all matrix-like types
3. **Flexible**: Support different output streams (cout, files, stringstream)
4. **Testable**: Easy to verify output in unit tests
5. **Customizable**: Allow formatting options
6. **Zero Duplication**: Avoid repeating printing logic
7. **No Dependencies**: Remove circular dependencies caused by print methods

## Proposed Solutions

### Solution 1: Template-Based operator<< with SFINAE (Recommended for C++17)

**Approach**: Create a single template `operator<<` that works for all matrix-like types using SFINAE (C++17 compatible).

**Key Design Decisions**:
- Uses SFINAE for C++17 compatibility (no C++20 concepts required)
- Accesses elements via `at_unsafe(row, col)` - works with any memory layout
- Future-proof for packed triangular matrices (only accesses valid elements)

**Implementation**:

```cpp
// include/trackingLib/math/linalg/matrix_io.h
#ifndef MATRIX_IO_H
#define MATRIX_IO_H

#include "base/first_include.h"
#include <ostream>
#include <iomanip>
#include <type_traits>

namespace tracking {
namespace math {

// SFINAE helper to detect matrix-like types
template <typename T, typename = void>
struct is_matrix_like : std::false_type {};

template <typename T>
struct is_matrix_like<T, std::void_t<
    decltype(std::declval<const T&>().at_unsafe(sint32{}, sint32{})),
    typename T::value_type,
    decltype(T::Rows),
    decltype(T::Cols)
>> : std::true_type {};

template <typename T>
inline constexpr bool is_matrix_like_v = is_matrix_like<T>::value;

// Template operator<< for all matrix-like types
template <typename M>
auto operator<<(std::ostream& os, const M& matrix)
    -> std::enable_if_t<is_matrix_like_v<M>, std::ostream&>
{
  using ValueType = typename M::value_type;
  
  // Access elements via at_unsafe() - works regardless of internal memory layout
  // This is future-proof for packed triangular matrices
  for (sint32 row = 0; row < M::Rows; ++row)
  {
    for (sint32 col = 0; col < M::Cols; ++col)
    {
      if constexpr (std::is_floating_point_v<ValueType>)
      {
        os << std::fixed << std::setprecision(6) << std::showpos
           << std::setw(12) << matrix.at_unsafe(row, col);
      }
      else
      {
        os << std::setw(8) << matrix.at_unsafe(row, col);
      }
      
      if (col < M::Cols - 1) {
        os << ", ";
      }
    }
    os << "\n";
  }
  return os;
}

} // namespace math
} // namespace tracking

#endif // MATRIX_IO_H
```

**Usage**:
```cpp
#include "trackingLib/math/linalg/matrix_io.h"

Matrix<float32, 3, 3> mat = ...;
std::cout << mat;  // Works!

std::ofstream file("matrix.txt");
file << mat;  // Works!

std::stringstream ss;
ss << mat;  // Works for testing!
```

**Pros**:
- Single implementation for all types
- Idiomatic C++
- Works with any `std::ostream`
- Easy to test
- No code duplication
- No circular dependencies

**Cons**:
- Requires all matrix types to have consistent interface
- Need to ensure `at_unsafe()`, `Rows`, `Cols` are available

### Solution 2: Formatter Class with Visitor Pattern

**Approach**: Create a `MatrixFormatter` class that can format any matrix type.

**Implementation**:

```cpp
// include/trackingLib/math/linalg/matrix_formatter.h
namespace tracking {
namespace math {

class MatrixFormatter {
public:
  struct Options {
    sint32 precision = 6;
    sint32 width = 12;
    bool showpos = true;
    bool fixed = true;
  };
  
  explicit MatrixFormatter(Options opts = {}) : options_(opts) {}
  
  template <typename MatrixType>
  void format(std::ostream& os, const MatrixType& matrix) const
  {
    using ValueType = typename MatrixType::value_type;
    
    for (sint32 row = 0; row < MatrixType::Rows; ++row)
    {
      for (sint32 col = 0; col < MatrixType::Cols; ++col)
      {
        if constexpr (std::is_floating_point_v<ValueType>)
        {
          if (options_.fixed) os << std::fixed;
          if (options_.showpos) os << std::showpos;
          os << std::setprecision(options_.precision) 
             << std::setw(options_.width) 
             << matrix.at_unsafe(row, col);
        }
        else
        {
          os << std::setw(options_.width) << matrix.at_unsafe(row, col);
        }
        
        if (col < MatrixType::Cols - 1) {
          os << ", ";
        }
      }
      os << "\n";
    }
  }
  
private:
  Options options_;
};

// Convenience operator<<
template <typename MatrixType>
std::ostream& operator<<(std::ostream& os, const MatrixType& matrix)
{
  MatrixFormatter formatter;
  formatter.format(os, matrix);
  return os;
}

} // namespace math
} // namespace tracking
```

**Usage**:
```cpp
Matrix<float32, 3, 3> mat = ...;

// Default formatting
std::cout << mat;

// Custom formatting
MatrixFormatter::Options opts;
opts.precision = 3;
opts.width = 8;
MatrixFormatter formatter(opts);
formatter.format(std::cout, mat);
```

**Pros**:
- Customizable formatting
- Single implementation
- Separation of concerns
- Easy to extend with new formats (JSON, CSV, etc.)

**Cons**:
- More complex than Solution 1
- Two ways to print (operator<< and formatter.format())

### Solution 3: Type Traits with Specialization

**Approach**: Use type traits to customize printing for different matrix types.

**Implementation**:

```cpp
// include/trackingLib/math/linalg/matrix_io.h
namespace tracking {
namespace math {

// Default printer for matrix-like types
template <typename MatrixType>
struct MatrixPrinter {
  static void print(std::ostream& os, const MatrixType& matrix)
  {
    using ValueType = typename MatrixType::value_type;
    
    for (sint32 row = 0; row < MatrixType::Rows; ++row)
    {
      for (sint32 col = 0; col < MatrixType::Cols; ++col)
      {
        if constexpr (std::is_floating_point_v<ValueType>)
        {
          os << std::fixed << std::setprecision(6) << std::showpos 
             << std::setw(12) << matrix.at_unsafe(row, col);
        }
        else
        {
          os << std::setw(8) << matrix.at_unsafe(row, col);
        }
        
        if (col < MatrixType::Cols - 1) {
          os << ", ";
        }
      }
      os << "\n";
    }
  }
};

// Specialization for DiagonalMatrix (only print diagonal)
template <typename ValueType, sint32 Size>
struct MatrixPrinter<DiagonalMatrix<ValueType, Size>> {
  static void print(std::ostream& os, const DiagonalMatrix<ValueType, Size>& matrix)
  {
    os << "Diagonal[";
    for (sint32 i = 0; i < Size; ++i)
    {
      os << matrix.at_unsafe(i);
      if (i < Size - 1) os << ", ";
    }
    os << "]\n";
  }
};

// Unified operator<<
template <typename MatrixType>
std::ostream& operator<<(std::ostream& os, const MatrixType& matrix)
{
  MatrixPrinter<MatrixType>::print(os, matrix);
  return os;
}

} // namespace math
} // namespace tracking
```

**Pros**:
- Can customize per type
- Single operator<< interface
- Efficient (compile-time dispatch)

**Cons**:
- Need specializations for each type that needs custom formatting
- More boilerplate

### Solution 4: CRTP Base Class for Printing (C++17 Compatible)

**Approach**: Create a CRTP base class that provides `operator<<`.

**Note**: This solution is fully C++17 compatible and doesn't require C++20 features.

**Implementation**:

```cpp
// include/trackingLib/math/linalg/printable_matrix.h
namespace tracking {
namespace math {

template <typename Derived>
class PrintableMatrix {
public:
  friend std::ostream& operator<<(std::ostream& os, const Derived& matrix)
  {
    using ValueType = typename Derived::value_type;
    
    for (sint32 row = 0; row < Derived::Rows; ++row)
    {
      for (sint32 col = 0; col < Derived::Cols; ++col)
      {
        if constexpr (std::is_floating_point_v<ValueType>)
        {
          os << std::fixed << std::setprecision(6) << std::showpos 
             << std::setw(12) << matrix.at_unsafe(row, col);
        }
        else
        {
          os << std::setw(8) << matrix.at_unsafe(row, col);
        }
        
        if (col < Derived::Cols - 1) {
          os << ", ";
        }
      }
      os << "\n";
    }
    return os;
  }
};

} // namespace math
} // namespace tracking
```

**Usage**:
```cpp
template <typename ValueType, sint32 Rows, sint32 Cols, bool IsRowMajor>
class Matrix : public PrintableMatrix<Matrix<ValueType, Rows, Cols, IsRowMajor>>
{
  // ... existing code
};
```

**Pros**:
- Automatic for all derived classes
- Single implementation
- No need to include separate header

**Cons**:
- Requires changing class hierarchy
- CRTP adds complexity
- May conflict with existing inheritance

## Recommended Solution

**Solution 1: Template-Based operator<< with SFINAE** is recommended because:

1. **C++17 Compatible**: Uses SFINAE, no C++20 features required
2. **Minimal Changes**: No need to modify existing class hierarchies
3. **Single Implementation**: One template works for all types
4. **Idiomatic C++**: Standard `operator<<` pattern
5. **Flexible**: Works with any `std::ostream`
6. **Testable**: Easy to test with `std::stringstream`
7. **No Dependencies**: Completely separate from matrix classes
8. **Future-Proof**: Works with any memory layout via `at_unsafe()` interface
9. **Zero Duplication**: Single implementation for all matrix types

## Implementation Plan

### Phase 1: Create matrix_io.h (2 hours)

**Actions**:
1. Create [`include/trackingLib/math/linalg/matrix_io.h`](include/trackingLib/math/linalg/matrix_io.h)
2. Implement template `operator<<` with SFINAE
3. Add formatting options (precision, width, etc.)
4. Add documentation

**Deliverables**:
- New header file with template operator<<
- Doxygen documentation

### Phase 2: Create Tests (2 hours)

**Actions**:
1. Create [`tests/math/test_matrix_io.cpp`](tests/math/test_matrix_io.cpp)
2. Test with `Matrix`, `SquareMatrix`, `DiagonalMatrix`, `TriangularMatrix`
3. Test with `std::stringstream` for output verification
4. Test with different value types (float32, float64, sint32)
5. Test formatting options

**Test Cases**:
```cpp
TEST(MatrixIO, BasicOutput) {
  Matrix<float32, 2, 2> mat = Matrix<float32, 2, 2>::FromList({
    {1.0f, 2.0f},
    {3.0f, 4.0f}
  });
  
  std::stringstream ss;
  ss << mat;
  
  std::string expected = 
    "     +1.000000,      +2.000000\n"
    "     +3.000000,      +4.000000\n";
  
  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, FileOutput) {
  Matrix<float32, 2, 2> mat = ...;
  
  std::ofstream file("test_matrix.txt");
  file << mat;
  file.close();
  
  // Verify file contents
}

TEST(MatrixIO, DiagonalMatrix) {
  DiagonalMatrix<float32, 3> diag = ...;
  
  std::stringstream ss;
  ss << diag;
  
  // Verify output
}
```

### Phase 3: Deprecate print() Methods (1 hour)

**Actions**:
1. Add deprecation warnings to all `print()` methods
2. Update documentation to recommend `operator<<`
3. Add migration examples

**Example Deprecation**:
```cpp
/// \brief Print the matrix to stdout
/// \deprecated Use operator<< instead: std::cout << matrix;
[[deprecated("Use operator<< instead: std::cout << matrix")]]
void print() const;
```

### Phase 4: Update Existing Code (2 hours)

**Actions**:
1. Find all uses of `print()` in codebase
2. Replace with `operator<<`
3. Update examples
4. Update documentation

**Search Pattern**:
```bash
grep -r "\.print()" include/ tests/ examples/
```

### Phase 5: Remove print() Methods (1 hour)

**Actions**:
1. Remove all `print()` method declarations from headers
2. Remove all `print()` method implementations
3. Remove from:
   - [`matrix.h`](include/trackingLib/math/linalg/matrix.h) / [`matrix.hpp`](include/trackingLib/math/linalg/matrix.hpp)
   - [`matrix_row_view.h`](include/trackingLib/math/linalg/matrix_row_view.h) / [`matrix_row_view.hpp`](include/trackingLib/math/linalg/matrix_row_view.hpp)
   - [`matrix_column_view.h`](include/trackingLib/math/linalg/matrix_column_view.h) / [`matrix_column_view.hpp`](include/trackingLib/math/linalg/matrix_column_view.hpp)
   - [`diagonal_matrix.h`](include/trackingLib/math/linalg/diagonal_matrix.h)
4. Verify all tests pass
5. Verify no compilation errors

### Phase 6: Documentation (1 hour)

**Actions**:
1. Update [`architecture.md`](.kilocode/rules/memory-bank/architecture.md)
2. Add examples to README
3. Update API documentation
4. Add migration guide

## Special Cases

### DiagonalMatrix

**Current Plan**: Convert to `SquareMatrix` for printing (creates circular dependency)

**New Approach**: Use template `operator<<` that accesses diagonal elements directly

**Implementation**:
```cpp
// In matrix_io.h, no special handling needed!
// The template operator<< will work automatically:

DiagonalMatrix<float32, 3> diag = ...;
std::cout << diag;  // Prints as full matrix with zeros

// If compact format desired, add specialization:
template <typename ValueType, sint32 Size>
std::ostream& operator<<(std::ostream& os, const DiagonalMatrix<ValueType, Size>& matrix)
{
  os << "Diagonal[";
  for (sint32 i = 0; i < Size; ++i)
  {
    os << matrix.at_unsafe(i);
    if (i < Size - 1) os << ", ";
  }
  os << "]";
  return os;
}
```

### CovarianceMatrixFactored

**Current**: Delegates to underlying matrix's `print()`

**New Approach**: `operator<<` will work automatically through delegation

**Implementation**:
```cpp
// No changes needed! operator<< will work:
CovarianceMatrixFactored<float32, 3> cov = ...;
std::cout << cov;  // Works through operator() conversion
```

### MatrixView

**Current**: Has its own `print()` implementation

**New Approach**: Template `operator<<` works automatically

**Benefit**: No need for separate implementation

## Advanced Features (Optional)

### Custom Formatting

```cpp
// matrix_io.h
namespace tracking {
namespace math {

struct MatrixFormatOptions {
  sint32 precision = 6;
  sint32 width = 12;
  bool showpos = true;
  bool fixed = true;
  const char* separator = ", ";
  const char* row_end = "\n";
};

// Global format options (thread_local for thread safety)
thread_local MatrixFormatOptions g_matrix_format_options;

// Manipulator for custom formatting
struct SetMatrixFormat {
  MatrixFormatOptions options;
};

inline SetMatrixFormat setMatrixFormat(MatrixFormatOptions opts) {
  return SetMatrixFormat{opts};
}

inline std::ostream& operator<<(std::ostream& os, const SetMatrixFormat& fmt) {
  g_matrix_format_options = fmt.options;
  return os;
}

} // namespace math
} // namespace tracking
```

**Usage**:
```cpp
MatrixFormatOptions opts;
opts.precision = 3;
opts.width = 8;

std::cout << setMatrixFormat(opts) << matrix;
```

### JSON/CSV Output

```cpp
// matrix_io.h
template <typename MatrixType>
std::string toJSON(const MatrixType& matrix) {
  std::stringstream ss;
  ss << "{\n  \"rows\": " << MatrixType::Rows << ",\n";
  ss << "  \"cols\": " << MatrixType::Cols << ",\n";
  ss << "  \"data\": [\n";
  // ... format as JSON array
  return ss.str();
}

template <typename MatrixType>
std::string toCSV(const MatrixType& matrix) {
  std::stringstream ss;
  for (sint32 row = 0; row < MatrixType::Rows; ++row) {
    for (sint32 col = 0; col < MatrixType::Cols; ++col) {
      ss << matrix.at_unsafe(row, col);
      if (col < MatrixType::Cols - 1) ss << ",";
    }
    ss << "\n";
  }
  return ss.str();
}
```

## Testing Strategy

### Unit Tests
- Test `operator<<` with all matrix types
- Test with different value types
- Test with different streams (cout, file, stringstream)
- Test formatting options
- Test special cases (empty matrices, 1x1, large matrices)

### Integration Tests
- Replace all `print()` calls in examples
- Verify output matches expected format
- Test in real usage scenarios

### Performance Tests
- Measure overhead of template instantiation
- Compare with old `print()` methods
- Ensure no runtime performance regression

## Migration Guide

### For Library Users

**Before**:
```cpp
Matrix<float32, 3, 3> mat = ...;
mat.print();  // Prints to stdout
```

**After**:
```cpp
Matrix<float32, 3, 3> mat = ...;
std::cout << mat;  // Idiomatic C++

// Or to file:
std::ofstream file("matrix.txt");
file << mat;

// Or to string:
std::stringstream ss;
ss << mat;
std::string str = ss.str();
```

### For Library Developers

**Before**:
```cpp
void someFunction() {
  Matrix<float32, 3, 3> mat = ...;
  mat.print();  // Debug output
}
```

**After**:
```cpp
void someFunction() {
  Matrix<float32, 3, 3> mat = ...;
  std::cout << mat;  // Debug output
  
  // Or with custom formatting:
  std::cout << std::setprecision(3) << mat;
}
```

## Timeline

- **Phase 1** (Create matrix_io.h): 2 hours
- **Phase 2** (Create tests): 2 hours
- **Phase 3** (Deprecate print()): 1 hour
- **Phase 4** (Update existing code): 2 hours
- **Phase 5** (Remove print()): 1 hour
- **Phase 6** (Documentation): 1 hour

**Total**: 9 hours

## Benefits

1. **Idiomatic C++**: Standard stream operators
2. **Zero Duplication**: Single template implementation
3. **Flexible**: Works with any stream
4. **Testable**: Easy to verify output
5. **Maintainable**: One place to update formatting
6. **No Dependencies**: Removes circular dependencies
7. **Extensible**: Easy to add JSON, CSV, etc.
8. **Consistent**: Same formatting across all types

## Risks and Mitigation

### Risk 1: Breaking Changes
**Mitigation**: 
- Deprecation period before removal
- Clear migration guide
- Update all examples

### Risk 2: Compilation Errors
**Mitigation**:
- Comprehensive testing
- Gradual rollout
- Keep print() deprecated but functional initially

### Risk 3: Performance Regression
**Mitigation**:
- Benchmark before/after
- Template instantiation is compile-time
- No runtime overhead expected

## Success Criteria

1. All matrix types support `operator<<`
2. All `print()` methods removed
3. All tests pass
4. No circular dependencies from printing
5. Documentation updated
6. Examples updated
7. No performance regression
8. Code duplication eliminated

## Future Compatibility: Triangular Matrix Memory Optimization

### Current State
[`TriangularMatrix`](include/trackingLib/math/linalg/triangular_matrix.h) currently inherits from [`SquareMatrix`](include/trackingLib/math/linalg/square_matrix.h), which stores the full N×N matrix even though only N(N+1)/2 elements are used.

### Future Optimization
**Goal**: Store only the lower/upper triangular elements in packed format.

**Memory Savings**:
- 3×3 matrix: 9 → 6 elements (33% savings)
- 6×6 matrix: 36 → 21 elements (42% savings)
- 10×10 matrix: 100 → 55 elements (45% savings)

### Why operator<< Solution is Future-Proof

The recommended `operator<<` solution accesses elements via `at_unsafe(row, col)`, which is an **interface method** that can be implemented differently for different memory layouts:

**Current Implementation** (full matrix storage):
```cpp
template <typename ValueType, sint32 Size, bool IsLower, bool IsRowMajor>
class TriangularMatrix : public SquareMatrix<ValueType, Size, IsRowMajor> {
  // at_unsafe(row, col) inherited from SquareMatrix
  // Accesses full N×N storage
};
```

**Future Implementation** (packed storage):
```cpp
template <typename ValueType, sint32 Size, bool IsLower>
class TriangularMatrix {
  // Packed storage: only N(N+1)/2 elements
  std::array<ValueType, Size * (Size + 1) / 2> _data;
  
  // at_unsafe() maps (row, col) to packed index
  ValueType at_unsafe(sint32 row, sint32 col) const {
    if constexpr (IsLower) {
      if (row < col) return ValueType{0};  // Upper part is zero
      // Map to packed index: row*(row+1)/2 + col
      return _data[row * (row + 1) / 2 + col];
    } else {
      if (row > col) return ValueType{0};  // Lower part is zero
      // Map to packed index for upper triangular
      return _data[col * (col + 1) / 2 + row];
    }
  }
};
```

**Key Point**: The `operator<<` implementation doesn't change! It still calls `at_unsafe(row, col)` for each element, and the triangular matrix's `at_unsafe()` handles the memory layout internally.

### Benefits of This Approach

1. **Decoupling**: Printing logic is independent of memory layout
2. **No Code Duplication**: Same `operator<<` works for all layouts
3. **Easy Migration**: Change memory layout without touching printing code
4. **Testability**: Can test printing independently of storage
5. **Flexibility**: Can have multiple storage strategies without duplicating print logic

### Example: Printing Packed Triangular Matrix

```cpp
// Future packed triangular matrix
TriangularMatrix<float32, 3, true> lower = ...;  // Packed storage

// operator<< still works!
std::cout << lower;

// Output (3x3 lower triangular):
//      +1.000000,      +0.000000,      +0.000000
//      +2.000000,      +3.000000,      +0.000000
//      +4.000000,      +5.000000,      +6.000000

// at_unsafe() internally:
// - Returns stored values for lower triangle
// - Returns 0 for upper triangle
// - No wasted memory!
```

### Migration Path for Triangular Matrix Optimization

When implementing packed storage in the future:

1. **No changes to matrix_io.h**: `operator<<` continues to work
2. **Update TriangularMatrix**:
   - Change internal storage to packed array
   - Update `at_unsafe()` to map indices correctly
   - Update constructors and other methods
3. **Tests still pass**: Printing tests verify output, not storage
4. **Zero impact on users**: API remains the same

This demonstrates why the template-based `operator<<` solution is the right choice for long-term maintainability.

## Next Steps

1. Review and approve this plan
2. Create feature branch
3. Implement Phase 1 (matrix_io.h)
4. Get feedback on implementation
5. Proceed with remaining phases
6. Merge to main branch
