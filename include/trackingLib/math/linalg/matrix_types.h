#ifndef D686C5CF_1160_4EDD_9688_B367417D1623
#define D686C5CF_1160_4EDD_9688_B367417D1623

/// \file matrix_types.h
/// \brief Type definitions and enumerations for matrix operations

namespace tracking
{
namespace math
{

/// \brief Specifies the memory layout order for matrix storage
///
/// This enum controls whether matrixes store elements in row-major order
/// (elements of a row are contiguous in memory) or column-major order
/// (elements of a column are contiguous in memory).
///
/// \note Row-major order is the default C/C++ array layout and is more
///       cache-friendly for row-wise operations. Column-major order is
///       used by some linear algebra libraries like LAPACK/BLAS.
///
/// \note Currently not used by the Matrix class, but reserved for future
///
/// \see Matrix for usage in matrix template parameters
enum class RowColumnOrder
{
  RowMajor,   ///< Elements are stored row by row (C/C++ default)
  ColumnMajor ///< Elements are stored column by column (FORTRAN/BLAS style)
};

/// \brief Specifies whether a triangular matrix is upper or lower triangular
///
/// This enum is used to specify the triangular structure of matrixes,
/// particularly for TriangularMatrix specializations.
///
/// \see TriangularMatrix for usage in template parameters
enum class UpLoType
{
  LowerTria, ///< Lower triangular matrix (elements above diagonal are zero)
  UpperTria  ///< Upper triangular matrix (elements below diagonal are zero)
};

} // namespace math
} // namespace tracking

#endif // D686C5CF_1160_4EDD_9688_B367417D1623
