#ifndef F7AF931D_0015_4C1A_B736_EB108A3CB8D5
#define F7AF931D_0015_4C1A_B736_EB108A3CB8D5

#include "conversions.h"
#include "math/linalg/square_matrix.hpp"     // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp" // IWYU pragma: keep

namespace tracking
{
namespace math
{
namespace conversions
{

/// \brief Creates a TriangularMatrix from a SquareMatrix
///
/// This function extracts the triangular part from a square matrix to create a triangular matrix.
/// The triangular structure (upper or lower) is determined by the IsLower_ template parameter.
///
/// \tparam ValueType_ The value type of matrix elements
/// \tparam Size_ The dimension of the matrices
/// \tparam IsLower_ Whether to extract lower triangular (true) or upper triangular (false) part
/// \tparam IsRowMajor_ The storage layout
/// \param[in] mat The source square matrix
/// \return TriangularMatrix containing the triangular part of the input matrix
/// \see TriangularFromList() for creating from initializer lists
/// \see DiagonalFromSquare() for extracting diagonal elements
template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularFromSquare(const SquareMatrix<ValueType_, Size_, IsRowMajor_>& mat)
    -> TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>
{
  return TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>{mat};
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // F7AF931D_0015_4C1A_B736_EB108A3CB8D5
