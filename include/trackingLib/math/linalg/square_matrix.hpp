#ifndef ADB29DD2_C5B0_4217_8728_B612EFF95F07
#define ADB29DD2_C5B0_4217_8728_B612EFF95F07

#include "gtest/gtest.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/square_matrix.h"

#include "math/linalg/diagonal_matrix.hpp"
#include "math/linalg/matrix_column_view.hpp"
#include "math/linalg/matrix_row_view.hpp"
//#include "math/linalg/matrix_view.hpp"
#include "math/linalg/triangular_matrix.h"
#include "math/linalg/triangular_matrix.hpp"
#include "math/linalg/vector.h"
#include "math/linalg/vector.hpp"
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace tracking::math
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
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::FromList(
    const std::initializer_list<std::initializer_list<ValueType_>>& list) -> SquareMatrix
{
  return SquareMatrix{Matrix::FromList(list)};
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
} // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::qrSolve(const SquareMatrix& b) const
    -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>
{
  const auto [Q, R] = householderQR();
  return static_cast<SquareMatrix<ValueType_, Size_, !IsRowMajor_>>(R.solve(Q.transpose() * b));
} // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::householderQR() const
    -> std::pair<SquareMatrix, TriangularMatrix<ValueType_, Size_, false, IsRowMajor_>>
{
  // implementation based on https://www.cs.cornell.edu/~bindel/class/cs6210-f09/lec18.pdf

  // Initially, Q is an identity matrix because no orthogonal transformations have been applied yet.
  SquareMatrix Q{SquareMatrix::Identity()};
  // Initializes the upper triangular matrix R as a copy of the input matrix. This is the matrix
  // that will be transformed to become upper triangular.
  SquareMatrix R{*this};

  // scale to reduce numerical issues
  const auto [min, max]  = R.minmax();
  const auto scaleFactor = std::abs(min) > std::abs(max) ? min : max;
  R /= scaleFactor;

  using ColumnVector = Vector<ValueType_, Size_>;
  static ColumnVector w{};
  for (auto j = 0; j < Size_; ++j)
  {
    // Extract Size_-j rows of the j-th column as a Vector starting in row j.
    w.setBlock(Size_ - j, 1, j, j, j, 0, R);
    for (auto k = 0; k < j; ++k)
    {
      // set unused values to zero
      w.at_unsafe(k) = static_cast<ValueType_>(0);
    }

    const ValueType_ normx = w.norm();
    // Determines the sign of the j-th diagonal element of R.
    const ValueType_ sign =
        (R.at_unsafe(j, j) < static_cast<ValueType_>(0)) ? static_cast<ValueType_>(1) : static_cast<ValueType_>(-1);
    const ValueType_ u1  = R.at_unsafe(j, j) - sign * normx;
    const ValueType_ tau = -sign * u1 / normx; // Computes the parameter tau for the Householder transformation.

    w /= u1;                                       // Computes the Householder vector w.
    w.at_unsafe(j)   = static_cast<ValueType_>(1); // Sets the j-th row of w to 1 for convenience.
    const auto wView = MatrixColumnView(w, 0, j);  // create view starting in j-th row

    // Update R using the Householder transformation
    for (auto i = j; i < Size_; ++i) // cols
    {
      // R(j:end, i) = R(j:end, i) - tau * w * (w' * R(j:end, i));
      const auto tau_dotRw = tau * (MatrixColumnView(R, i, j) * wView);
      for (auto k = j; k < Size_; ++k) // rows
      {
        R.at_unsafe(k, i) -= tau_dotRw * wView.at_unsafe(k - j);
      }
    }

    // Update Q using the Householder transformation
    for (auto i = 0; i < Size_; ++i) // rows
    {
      // Q(i,j:end) = Q(i,j:end) - tau * (Q(i,j:end) * w) * w';
      const auto tau_dotQw = tau * (MatrixRowView(Q, i, j) * wView);
      for (auto k = j; k < Size_; ++k) // cols
      {
        Q.at_unsafe(i, k) -= tau_dotQw * wView.at_unsafe(k - j);
      }
    }
  }
  auto triuR = TriangularMatrix<ValueType_, Size_, false, IsRowMajor_>{std::move(R)};
  triuR *= scaleFactor;
  return std::make_pair(std::move(Q), std::move(triuR));
} // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::decomposeLLT() const
    -> tl::expected<TriangularMatrix<ValueType_, Size_, true, IsRowMajor_>, Errors>
{
  if (isSymmetric())
  {
    if (hasStrictlyPositiveDiagonalElems())
    {
      TriangularMatrix<ValueType_, Size_, true, IsRowMajor_> L{};
      for (auto j = 0; j < Size_; ++j)
      {
        ValueType_ sum = this->at_unsafe(j, j);
        for (auto k = 0; k < j; ++k)
        {
          sum -= L.at_unsafe(j, k) * L.at_unsafe(j, k);
        }
        L.at_unsafe(j, j) = std::sqrt(sum);

        for (auto i = j + 1; i < Size_; ++i)
        {
          sum = this->at_unsafe(i, j);
          for (auto k = 0; k < j; ++k)
          {
            sum -= L.at_unsafe(i, k) * L.at_unsafe(j, k);
          }
          L.at_unsafe(i, j) = sum / L.at_unsafe(j, j);
        }
      }
      return std::move(L);
    }
    return tl::unexpected<Errors>{Errors::matrix_not_positive_definite};
  }
  return tl::unexpected<Errors>{Errors::matrix_not_symmetric};
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::decomposeLDLT() const
    -> tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, true, IsRowMajor_>, DiagonalMatrix<ValueType_, Size_>>, Errors>
{
  if (isSymmetric())
  {
    if (hasStrictlyPositiveDiagonalElems())
    {
      TriangularMatrix<ValueType_, Size_, true, IsRowMajor_> L{};
      DiagonalMatrix<ValueType_, Size_>                      D{};
      for (auto j = 0; j < Size_; ++j)
      {
        ValueType_ sum = this->at_unsafe(j, j);
        for (auto k = 0; k < j; ++k)
        {
          sum -= D.at_unsafe(k) * L.at_unsafe(j, k) * L.at_unsafe(j, k);
        }
        D.at_unsafe(j)    = sum;
        L.at_unsafe(j, j) = static_cast<ValueType_>(1);

        for (auto i = j + 1; i < Size_; ++i)
        {
          sum = this->at_unsafe(i, j);
          for (auto k = 0; k < j; ++k)
          {
            sum -= D.at_unsafe(k) * L.at_unsafe(i, k) * L.at_unsafe(j, k);
          }
          L.at_unsafe(i, j) = sum / D.at_unsafe(j);
        }
      }
      return std::make_pair(std::move(L), std::move(D));
    }
    return tl::unexpected<Errors>{Errors::matrix_not_positive_definite};
  }
  return tl::unexpected<Errors>{Errors::matrix_not_symmetric};
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::decomposeUDUT() const
    -> tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, false, IsRowMajor_>, DiagonalMatrix<ValueType_, Size_>>, Errors>
{
  // Grewal & Andrews, Kalman Filtering Theory and Practice Using MATLAB, 4th
  // Edition, Wiley, 2014.
  //
  // Performs modified Cholesky decomposition of symmetric positive-definite
  // matrix P (input).

  if (isSymmetric())
  {
    const auto&                                             P = this->_data;
    TriangularMatrix<ValueType_, Size_, false, IsRowMajor_> U{};
    DiagonalMatrix<ValueType_, Size_>                       D{};
    for (sint32 j = Size_ - 1; j >= 0; --j)
    {
      for (sint32 i = j; i >= 0; --i)
      {
        auto sigma = P(i, j);
        for (sint32 k = j + 1; k < Size_; ++k)
        {
          sigma -= U(i, k) * D[k] * U(j, k);
        }
        if (i == j)
        {
          D[j]    = std::max(sigma, std::numeric_limits<ValueType_>::epsilon());
          U(j, j) = static_cast<ValueType_>(1.0);
        }
        else
        {
          U(i, j) = sigma / D[j];
        }
      }
    }
    return std::make_pair(std::move(U), std::move(D));
  }
  return tl::unexpected<Errors>{Errors::matrix_not_symmetric};
}

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::inverse() const -> SquareMatrix<ValueType_, Size_, !IsRowMajor_>
{
  return qrSolve(SquareMatrix::Identity());
} // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::isSymmetric() const -> bool
{
  // check all off diagonal elements
  for (auto row = 1; row < Size_; ++row)
  {
    for (auto col = row; col < Size_; ++col)
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

} // namespace tracking::math

#endif // ADB29DD2_C5B0_4217_8728_B612EFF95F07
