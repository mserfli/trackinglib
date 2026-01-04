/// \file conversions.h
/// \brief Centralized conversion system for matrix types
///
/// This header provides forward declarations for all matrix types used in the conversion system.
/// The conversion system is designed to eliminate circular dependencies between matrix classes
/// by providing centralized conversion functions with a consistent `<target>From<source>` naming convention.
///
/// \see conversions/matrix_conversions.hpp for Matrix conversion functions
/// \see conversions/diagonal_conversions.hpp for DiagonalMatrix conversion functions
/// \see conversions/square_conversions.hpp for SquareMatrix conversion functions
/// \see conversions/triangular_conversions.hpp for TriangularMatrix conversion functions
/// \see conversions/vector_conversions.hpp for Vector conversion functions
/// \see conversions/covariance_matrix_conversions.hpp for CovarianceMatrix conversion functions

#ifndef CB42095E_E918_42A9_BBED_1299E4CDDFAD
#define CB42095E_E918_42A9_BBED_1299E4CDDFAD

#include "base/first_include.h" // IWYU pragma: keep

// Forward declarations for all matrix types used in conversions
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
