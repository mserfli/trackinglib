#ifndef C5DC5CCE_5C5B_4EAF_813E_3F6FEDDF09FA
#define C5DC5CCE_5C5B_4EAF_813E_3F6FEDDF09FA

#include "math/linalg/diagonal_matrix.hpp"
#include "math/linalg/matrix_column_view.hpp"
#include "math/linalg/matrix_row_view.hpp" // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp"

namespace tracking
{
namespace math
{

// Forward declaration to prevent cyclic includes
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix;

// Householder QR decomposition
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

    w /= u1;                                                                  // Computes the Householder vector w.
    w.at_unsafe(j)   = static_cast<ValueType_>(1);                            // Sets the j-th row of w to 1 for convenience.
    const auto wView = MatrixColumnView<ValueType_, Size_, 1, true>(w, 0, j); // create view starting in j-th row

    // Update R using the Householder transformation
    for (auto i = j; i < Size_; ++i) // cols
    {
      // R(j:end, i) = R(j:end, i) - tau * w * (w' * R(j:end, i));
      const auto tau_dotRw = tau * (MatrixColumnView<ValueType_, Size_, Size_, IsRowMajor_>(R, i, j) * wView);
      for (auto k = j; k < Size_; ++k) // rows
      {
        R.at_unsafe(k, i) -= tau_dotRw * wView.at_unsafe(k - j);
      }
    }

    // Update Q using the Householder transformation
    for (auto i = 0; i < Size_; ++i) // rows
    {
      // Q(i,j:end) = Q(i,j:end) - tau * (Q(i,j:end) * w) * w';
      const auto tau_dotQw = tau * (MatrixRowView<ValueType_, Size_, Size_, IsRowMajor_>(Q, i, j) * wView);
      for (auto k = j; k < Size_; ++k) // cols
      {
        Q.at_unsafe(i, k) -= tau_dotQw * wView.at_unsafe(k - j);
      }
    }
  }
  auto triuR = TriangularMatrix<ValueType_, Size_, false, IsRowMajor_>{std::move(R)};
  triuR *= scaleFactor;
  return std::make_pair(std::move(Q), std::move(triuR));
}

// LLT decomposition
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::decomposeLLT() const
    -> tl::expected<TriangularMatrix<ValueType_, Size_, true, IsRowMajor_>, Errors>
{
  if (hasStrictlyPositiveDiagonalElems()) // fail fast
  {
    if (isSymmetric()) // fail fast
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
    return tl::unexpected<Errors>{Errors::matrix_not_symmetric};
  }
  return tl::unexpected<Errors>{Errors::matrix_not_positive_definite};
}

// LDLT decomposition
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

// UDUT decomposition
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareMatrix<ValueType_, Size_, IsRowMajor_>::decomposeUDUT() const
    -> tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, false, IsRowMajor_>, DiagonalMatrix<ValueType_, Size_>>, Errors>
{
  if (isSymmetric())
  {
    const auto&                                             P = *this;
    TriangularMatrix<ValueType_, Size_, false, IsRowMajor_> U{};
    DiagonalMatrix<ValueType_, Size_>                       D{};
    for (sint32 j = Size_ - 1; j >= 0; --j)
    {
      for (sint32 i = j; i >= 0; --i)
      {
        auto sigma = P.at_unsafe(i, j);
        for (sint32 k = j + 1; k < Size_; ++k)
        {
          sigma -= U.at_unsafe(i, k) * D.at_unsafe(k) * U.at_unsafe(j, k);
        }
        if (i == j)
        {
          D.at_unsafe(j)    = std::max(sigma, std::numeric_limits<ValueType_>::epsilon());
          U.at_unsafe(j, j) = static_cast<ValueType_>(1.0);
        }
        else
        {
          U.at_unsafe(i, j) = sigma / D.at_unsafe(j);
        }
      }
    }
    return std::make_pair(std::move(U), std::move(D));
  }
  return tl::unexpected<Errors>{Errors::matrix_not_symmetric};
}

} // namespace math
} // namespace tracking

#endif // C5DC5CCE_5C5B_4EAF_813E_3F6FEDDF09FA
