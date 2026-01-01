#ifndef CB42095E_E918_42A9_BBED_1299E4CDDFAD
#define CB42095E_E918_42A9_BBED_1299E4CDDFAD

#include "base/first_include.h" // IWYU pragma: keep

// Forward declarations for all matrix types
namespace tracking
{
namespace math
{

// Matrix types
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix;

// Diagonal matrix
template <typename ValueType_, sint32 Size_>
class DiagonalMatrix;

// Square matrix
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix;

// Triangular matrix
template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix;

// Vector
template <typename ValueType_, sint32 Size_>
class Vector;

// Views
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class MatrixColumnView;

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class MatrixRowView;

} // namespace math
} // namespace tracking

#endif // CB42095E_E918_42A9_BBED_1299E4CDDFAD
