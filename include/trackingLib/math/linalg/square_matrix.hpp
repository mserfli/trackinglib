#ifndef ADB29DD2_C5B0_4217_8728_B612EFF95F07
#define ADB29DD2_C5B0_4217_8728_B612EFF95F07

#include "math/linalg/square_matrix.h"

#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/triangular_matrix.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
SquareMatrix<FloatType, Size>::SquareMatrix(const Matrix<FloatType, Size, Size>& other)
    : Matrix<FloatType, Size, Size>{other}
{
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::qrSolve(SquareMatrix<FloatType, Size>& x, const SquareMatrix<FloatType, Size>& b) const
    -> bool
{
  Eigen::HouseholderQR<Eigen::Matrix<FloatType, Size, Size>> qr(this->_data);
  x._data = qr.solve(b._data);
  return true;
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::decomposeLLT(TriangularMatrix<FloatType, Size, true>& L) const -> bool
{
  const Eigen::LLT<Eigen::Matrix<FloatType, Size, Size>> llt(this->_data);

  bool isOk = (llt.info() == Eigen::Success);
  if (isOk)
  {
    L.setZeros();
    const auto& internalL = llt.matrixL();
    // copy lower triangular elements of internal matrix
    for (sint32 col = 0; col < Size; ++col)
    {
      L(col, col) = internalL(col, col);
      for (sint32 row = col + 1; row < Size; ++row)
      {
        L(row, col) = internalL(row, col);
      }
    }
  }
  return isOk;
}

template <typename FloatType, sint32 Size>
inline void SquareMatrix<FloatType, Size>::setIdentity()
{
  this->_data.setIdentity();
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::Identity() -> SquareMatrix
{
  SquareMatrix tmp;
  tmp.setIdentity();
  return tmp;
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::decomposeLDLT(TriangularMatrix<FloatType, Size, true>& L,
                                                         DiagonalMatrix<FloatType, Size>&         D) const -> bool
{
  const Eigen::SimplicialLDLT<Eigen::SparseMatrix<FloatType>, Eigen::Lower, Eigen::NaturalOrdering<int>> ldlt(
      this->_data.sparseView());

  bool isOk(ldlt.info() == Eigen::Success);
  if (isOk)
  {
    L.setIdentity();

    const auto  internalL = Eigen::SparseMatrix<FloatType>(ldlt.matrixL());
    const auto& internalD = ldlt.vectorD();
    // copy strict lower triangular and diagonal elements of internal matrix
    for (sint32 col = 0; col < Size; ++col)
    {
      D[col] = internalD[col];
      for (sint32 row = col + 1; row < Size; ++row)
      {
        L(row, col) = internalL.coeff(row, col);
      }
    }
  }
  return isOk;
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::decomposeUDUT(TriangularMatrix<FloatType, Size, false>& U,
                                                         DiagonalMatrix<FloatType, Size>&          D) const -> bool
{
  // Grewal & Andrews, Kalman Filtering Theory and Practice Using MATLAB, 4th
  // Edition, Wiley, 2014.
  //
  // Performs modified Cholesky decomposition of symmetric positive-definite
  // matrix P (input).

  auto P = static_cast<FloatType>(0.5) * (this->_data + this->_data.transpose());
  for (sint32 j = Size - 1; j >= 0; --j)
  {
    for (sint32 i = j; i >= 0; --i)
    {
      auto sigma = P(i, j);
      for (sint32 k = j + 1; k < Size; ++k)
      {
        sigma -= U(i, k) * D[k] * U(j, k);
      }
      if (i == j)
      {
        D[j]    = std::max(sigma, std::numeric_limits<FloatType>::epsilon());
        U(j, j) = static_cast<FloatType>(1.0);
      }
      else
      {
        U(i, j) = sigma / D[j];
      }
    }
  }
  return true;
}

template <typename FloatType, sint32 Size>
inline auto SquareMatrix<FloatType, Size>::inverse() const -> SquareMatrix
{
  SquareMatrix inv{};
  auto isOk = qrSolve(inv, SquareMatrix::Identity());
  assert(isOk);
  return inv;
}

} // namespace math
} // namespace tracking

#endif // ADB29DD2_C5B0_4217_8728_B612EFF95F07
