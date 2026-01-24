#ifndef A000E27D_C91E_4768_A8FD_B292AB7B986A
#define A000E27D_C91E_4768_A8FD_B292AB7B986A

#include "conversions.h"
#include "math/linalg/diagonal_matrix.hpp" // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"   // IWYU pragma: keep

namespace tracking
{
namespace math
{
namespace conversions
{

/// \brief Creates a DiagonalMatrix from the diagonal elements of a SquareMatrix
///
/// This function extracts the diagonal elements from a square matrix to create a diagonal matrix.
/// All off-diagonal elements are discarded, preserving only the main diagonal values.
///
/// \tparam ValueType_ The atomic data type of internal elements
/// \tparam Size_ The size of the square matrix and resulting diagonal matrix
/// \tparam IsRowMajor_ The storage layout of the source matrix
/// \param[in] mat The source square matrix
/// \return DiagonalMatrix containing the diagonal elements of the input matrix
/// \see DiagonalFromList() for creating diagonal matrixes from initializer lists
/// \see SquareFromDiagonal() for the reverse conversion
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto DiagonalFromSquare(const SquareMatrix<ValueType_, Size_, IsRowMajor_>& mat) -> DiagonalMatrix<ValueType_, Size_>
{
  DiagonalMatrix<ValueType_, Size_> result;
  for (sint32 i = 0; i < Size_; ++i)
  {
    result.at_unsafe(i) = mat.at_unsafe(i, i);
  }
  return result;
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // A000E27D_C91E_4768_A8FD_B292AB7B986A
