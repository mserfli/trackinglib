#ifndef C21D598C_CF10_4BAC_8857_B0DA4A653638
#define C21D598C_CF10_4BAC_8857_B0DA4A653638

#include "conversions.h"
#include "math/linalg/diagonal_matrix.hpp" // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"   // IWYU pragma: keep

namespace tracking
{
namespace math
{
namespace conversions
{

/// \brief Creates a SquareMatrix from a DiagonalMatrix
///
/// This function creates a square matrix with the diagonal elements from a diagonal matrix.
/// All off-diagonal elements are set to zero.
///
/// \tparam ValueType_ The value type of matrix elements
/// \tparam Size_ The dimension of the matrices
/// \tparam IsRowMajor_ The storage layout of the resulting square matrix
/// \param[in] diag The source diagonal matrix
/// \return SquareMatrix with diagonal elements from the input and zeros elsewhere
/// \see DiagonalFromSquare() for the reverse conversion
/// \see SquareFromList() for creating from initializer lists
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareFromDiagonal(const DiagonalMatrix<ValueType_, Size_>& diag) -> SquareMatrix<ValueType_, Size_, IsRowMajor_>
{
  SquareMatrix<ValueType_, Size_, IsRowMajor_> result{};
  for (sint32 i = 0; i < Size_; ++i)
  {
    result.at_unsafe(i, i) = diag.at_unsafe(i);
  }
  return result;
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // C21D598C_CF10_4BAC_8857_B0DA4A653638
