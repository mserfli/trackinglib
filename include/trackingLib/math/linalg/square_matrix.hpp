#ifndef ADB29DD2_C5B0_4217_8728_B612EFF95F07
#define ADB29DD2_C5B0_4217_8728_B612EFF95F07

#include "math/linalg/square_matrix.h"

#include "math/linalg/diagonal_matrix.hpp"              // IWYU pragma: keep
#include "math/linalg/matrix_column_view.hpp"           // IWYU pragma: keep
#include "math/linalg/matrix_row_view.hpp"              // IWYU pragma: keep
#include "math/linalg/square_matrix_decompositions.hpp" // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/vector.hpp"                       // IWYU pragma: keep
#include <cmath>                                        // sqrt
#include <limits>

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
SquareMatrix<ValueType_, Size_, IsRowMajor_>::SquareMatrix(const DiagonalMatrix<ValueType_, Size_>& other)
{
  for (auto idx = 0; idx < Size_; ++idx)
  {
    this->at_unsafe(idx, idx) = other.at_unsafe(idx);
  }
}


template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline void SquareMatrix<ValueType_, Size_, IsRowMajor_>::setIdentity()
{
  *this = SquareMatrix{DiagonalMatrix<ValueType_, Size_>::Identity()};
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::Identity() -> SquareMatrix
{
  return SquareMatrix{DiagonalMatrix<ValueType_, Size_>::Identity()};
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::qrSolve(const SquareMatrix& b) const
    -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>
{
  const auto [Q, R] = householderQR();
  return static_cast<SquareMatrix<ValueType_, Size_, !IsRowMajor_>>(R.solve(Q.transpose() * b));
}


template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::inverse() const -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>
{
  return qrSolve(SquareMatrix::Identity());
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::isSymmetric() const -> bool
{
  // check all off diagonal elements
  for (auto row = 0; row < Size_; ++row)
  {
    for (auto col = row + 1; col < Size_; ++col)
    {
      const auto absDiff = std::abs(this->at_unsafe(row, col) - this->at_unsafe(col, row));
      if (absDiff > std::numeric_limits<ValueType_>::epsilon())
      {
        return false;
      }
    }
  }
  return true;
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline void SquareMatrix<ValueType_, Size_, IsRowMajor_>::symmetrize()
{
  *this += this->transpose();
  *this *= static_cast<ValueType_>(0.5);
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::hasStrictlyPositiveDiagonalElems() const -> bool
{
  sint32 j{0};
  // check all diagonal elements
  while ((j < Size_) && (this->at_unsafe(j, j) > static_cast<ValueType_>(0)))
  {
    ++j;
  }
  return j == Size_;
}

} // namespace math
} // namespace tracking

#endif // ADB29DD2_C5B0_4217_8728_B612EFF95F07
