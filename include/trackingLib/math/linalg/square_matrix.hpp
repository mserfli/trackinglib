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
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::FromList(
    const std::initializer_list<std::initializer_list<ValueType_>>& list) -> SquareMatrix
{
  return SquareMatrix{BaseMatrix::FromList(list)};
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::trace() const -> ValueType_
{
  ValueType_ result{0};
  for (auto i = 0; i < Size_; ++i)
  {
    result += this->at_unsafe(i, i);
  }
  return result;
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::determinant() const -> ValueType_
{
  // Create a copy of the matrix for LU decomposition
  SquareMatrix<ValueType_, Size_, IsRowMajor_> luMatrix{*this};
  sint32                                       permutationCount{0};

  // Perform LU decomposition with partial pivoting
  for (auto k = 0; k < Size_; ++k)
  {
    // Partial pivoting: find the row with maximum element in current column
    auto       maxRow = k;
    ValueType_ maxVal = std::abs(luMatrix.at_unsafe(k, k));

    for (auto i = k + 1; i < Size_; ++i)
    {
      const auto absVal = std::abs(luMatrix.at_unsafe(i, k));
      if (absVal > maxVal)
      {
        maxVal = absVal;
        maxRow = i;
      }
    }

    // Swap rows if necessary
    if (maxRow != k)
    {
      for (auto j = 0; j < Size_; ++j)
      {
        std::swap(luMatrix.at_unsafe(k, j), luMatrix.at_unsafe(maxRow, j));
      }
      permutationCount++;
    }

    // Check for singular matrix
    if (std::abs(luMatrix.at_unsafe(k, k)) < std::numeric_limits<ValueType_>::epsilon())
    {
      return static_cast<ValueType_>(0);
    }

    // Perform elimination for rows below the current one
    for (auto i = k + 1; i < Size_; ++i)
    {
      const auto factor = luMatrix.at_unsafe(i, k) / luMatrix.at_unsafe(k, k);

      for (auto j = k; j < Size_; ++j)
      {
        luMatrix.at_unsafe(i, j) -= factor * luMatrix.at_unsafe(k, j);
      }
    }
  }

  // Calculate determinant as product of diagonal elements, multiplied by (-1)^permutationCount
  ValueType_ det{1};
  for (auto i = 0; i < Size_; ++i)
  {
    det *= luMatrix.at_unsafe(i, i);
  }

  // Apply sign based on number of row permutations
  if (permutationCount % 2 != 0)
  {
    det = -det;
  }

  return det;
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
inline void SquareMatrix<ValueType_, Size_, IsRowMajor_>::symmetrize()
{
  *this += this->transpose();
  *this *= static_cast<ValueType_>(0.5);
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::isSymmetric(ValueType_ tolerance) const -> bool
{
  // check all off diagonal elements
  for (auto row = 0; row < Size_; ++row)
  {
    for (auto col = row + 1; col < Size_; ++col)
    {
      const auto absDiff = std::abs(this->at_unsafe(row, col) - this->at_unsafe(col, row));
      if (absDiff > tolerance)
      {
        return false;
      }
    }
  }
  return true;
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::isPositiveDefinite() const -> bool
{
  // Try Cholesky decomposition - if it succeeds, matrix is positive definite
  const auto choleskyResult = this->decomposeLLT();
  const bool result         = choleskyResult.has_value();
  return result;
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::isPositiveSemiDefinite() const -> bool
{
  // we can only use Cholesky decomposition which has more strict checks
  return isPositiveDefinite();
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

// ============================================================================
// Matrix Property Check Functions
// ============================================================================

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::isOrthogonal(ValueType_ tolerance) const -> bool
{
  const auto QtQ      = this->transpose() * (*this);
  const auto identity = SquareMatrix<ValueType_, Size_, !IsRowMajor_>::Identity();

  for (auto i = 0; i < Size_; ++i)
  {
    for (auto j = 0; j < Size_; ++j)
    {
      const auto diff = std::abs(QtQ.at_unsafe(i, j) - identity.at_unsafe(i, j));
      if (diff > tolerance)
      {
        return false;
      }
    }
  }
  return true;
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::isUpperTriangular(ValueType_ tolerance) const -> bool
{
  for (auto i = 0; i < Size_; ++i)
  {
    for (auto j = 0; j < i; ++j)
    {
      if (std::abs(this->at_unsafe(i, j)) > tolerance)
      {
        return false;
      }
    }
  }
  return true;
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::isLowerTriangular(ValueType_ tolerance) const -> bool
{
  for (auto i = 0; i < Size_; ++i)
  {
    for (auto j = i + 1; j < Size_; ++j)
    {
      if (std::abs(this->at_unsafe(i, j)) > tolerance)
      {
        return false;
      }
    }
  }
  return true;
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::hasUnitDiagonal(ValueType_ tolerance) const -> bool
{
  for (auto i = 0; i < Size_; ++i)
  {
    if (std::abs(this->at_unsafe(i, i) - static_cast<ValueType_>(1.0)) > tolerance)
    {
      return false;
    }
  }
  return true;
}

} // namespace math
} // namespace tracking

#endif // ADB29DD2_C5B0_4217_8728_B612EFF95F07
