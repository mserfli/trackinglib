#ifndef F7AF931D_0015_4C1A_B736_EB108A3CB8D5
#define F7AF931D_0015_4C1A_B736_EB108A3CB8D5

#include "conversions.h"
#include "math/linalg/square_matrix.hpp"     // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp" // IWYU pragma: keep
#include <initializer_list>

namespace tracking
{
namespace math
{
namespace conversions
{

// TriangularFromList: TriangularMatrix from initializer_list<initializer_list<ValueType_>>
template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularFromList(const std::initializer_list<std::initializer_list<ValueType_>>& list)
    -> TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>
{
  return TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>{SquareMatrix<ValueType_, Size_, IsRowMajor_>::FromList(list)};
}

// TriangularFromSquare: TriangularMatrix from SquareMatrix
// <target>From<source> pattern
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
