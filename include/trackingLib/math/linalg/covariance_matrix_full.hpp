#ifndef E4324F70_8D26_4E47_B84E_1A44466A0FBB
#define E4324F70_8D26_4E47_B84E_1A44466A0FBB

#include "math/linalg/covariance_matrix_full.h"

#include "math/linalg/square_matrix.hpp"
#include "math/linalg/triangular_matrix.hpp"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
CovarianceMatrixFull<FloatType, Size>::CovarianceMatrixFull(const SquareMatrix<FloatType, Size>& other, const bool isInverse)
    : SquareMatrix<FloatType, Size>{other}
    , _isInverse{isInverse}
{
  assert(this->isSymmetric() && "Constructed covariance not symmetric");
}

template <typename FloatType, sint32 Size>
auto CovarianceMatrixFull<FloatType, Size>::Identity() -> CovarianceMatrixFull
{
  CovarianceMatrixFull cov{SquareMatrix<FloatType, Size>::Identity(), false};
  return cov;
}

template <typename FloatType, sint32 Size>
void CovarianceMatrixFull<FloatType, Size>::setIdentity()
{
  SquareMatrix<FloatType, Size>::setIdentity();
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFull<FloatType, Size>::operator()(sint32 row, sint32 col) const -> FloatType
{
  return SquareMatrix<FloatType, Size>::operator()(row, col);
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFull<FloatType, Size>::inverse() const -> CovarianceMatrixFull
{
  CovarianceMatrixFull                    inv{};
  TriangularMatrix<FloatType, Size, true> L{};

  auto isOk = SquareMatrix<FloatType, Size>::decomposeLLT(L);
  assert(isOk && "matrix not positive definite");

  // L*(L'*Ainv) = eye(n,n)
  // L*u = eye(n,n)  -> solve for u using forward substitution on each column vector of eye(n,n)
  auto u = L.solve(CovarianceMatrixFull::Identity());
  // L'*Ainv = u     -> solve for Ainv using backward substitution
  auto s=L.transpose().solve(u);
  inv = CovarianceMatrixFull(static_cast<FloatType>(0.5)*(s+s.transpose()), !_isInverse);

  return inv;
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFull<FloatType, Size>::isInverse() const -> bool
{
  return _isInverse;
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFull<FloatType, Size>::apaT(const SquareMatrix<FloatType, Size>& A)
{
  assert(this->isSymmetric() && "Covariance currently not symmetric");
  if (_isInverse)
  {
    SquareMatrix<FloatType, Size> res = A.transpose() * this->operator*=(A);
    res+=res.transpose();
    res*=static_cast<FloatType>(0.5);
    this->operator=(CovarianceMatrixFull(res, _isInverse));
  }
  else
  {
    SquareMatrix<FloatType, Size> res = A * this->operator*=(A.transpose());
    res+=res.transpose();
    res*=static_cast<FloatType>(0.5);
    this->operator=(CovarianceMatrixFull(res, _isInverse));
  }
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFull<FloatType, Size>::apaT(const SquareMatrix<FloatType, Size>& A) const -> CovarianceMatrixFull
{
  auto copy(*this);
  copy.apaT(A);
  return copy;
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFull<FloatType, Size>::setVariance(const sint32 idx, const FloatType val)
{
  constexpr auto zero = static_cast<FloatType>(0.0);
  for (sint32 j = 0; j < Size; ++j)
  {
    SquareMatrix<FloatType, Size>::operator()(idx, j) = zero;
    SquareMatrix<FloatType, Size>::operator()(j, idx) = zero;
  }
  SquareMatrix<FloatType, Size>::operator()(idx, idx) = val;
}

} // namespace math
} // namespace tracking

#endif // E4324F70_8D26_4E47_B84E_1A44466A0FBB
