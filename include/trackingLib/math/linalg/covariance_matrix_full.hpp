#ifndef E4324F70_8D26_4E47_B84E_1A44466A0FBB
#define E4324F70_8D26_4E47_B84E_1A44466A0FBB

#include "math/linalg/covariance_matrix_full.h"

#include "math/linalg/errors.h"
#include "math/linalg/square_matrix.hpp"     // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename FloatType_, sint32 Size_>
auto CovarianceMatrixFull<FloatType_, Size_>::FromDiagonal(const DiagonalMatrix<FloatType_, Size_>& diag) -> CovarianceMatrixFull
{
  assert(diag.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
  return CovarianceMatrixFull{diag};
}

template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFull<FloatType_, Size_>::inverse() const -> tl::expected<CovarianceMatrixFull, Errors>
{
  const auto retVal = BaseSquareMatrix::decomposeLLT();
  if (retVal.has_value())
  {
    const auto& L = *retVal;
    // A * Ainv = eye(n,n) with A=L*L' from Cholesky decomposition
    // L*(L'*Ainv) = eye(n,n)
    // L*u = eye(n,n)  -> solve for u using forward substitution on each column vector of eye(n,n)
    const auto u = L.solve(CovarianceMatrixFull::Identity());
    // L'*Ainv = u     -> solve for Ainv using backward substitution
    math::SquareMatrix<FloatType_, Size_, true> cov{L.transpose().solve(u)};
    cov.symmetrize();
    return CovarianceMatrixFull{std::move(cov)};
  }
  return tl::unexpected<Errors>{retVal.error()};
}

template <typename FloatType_, sint32 Size_>
template <bool IsRowMajor_>
inline void CovarianceMatrixFull<FloatType_, Size_>::apaT(const tracking::math::SquareMatrix<FloatType_, Size_, IsRowMajor_>& A)
{
  assert(this->isSymmetric() && "Covariance currently not symmetric");
  // TODO(matthias): optimization - calculate only the upper triangle part of P and fill lower triangle part
  // for normal covariance matrix P, the calculation is P = A*P*A'
  const auto       paT = this->operator*(A.transpose());
  BaseSquareMatrix cov{A.operator*(paT)};
  cov.symmetrize();
  *this = CovarianceMatrixFull{std::move(cov)};
}

template <typename FloatType_, sint32 Size_>
template <bool IsRowMajor_>
inline auto CovarianceMatrixFull<FloatType_, Size_>::apaT(
    const tracking::math::SquareMatrix<FloatType_, Size_, IsRowMajor_>& A) const -> CovarianceMatrixFull
{
  auto copy(*this);
  copy.apaT(A);
  return copy;
}

template <typename FloatType_, sint32 Size_>
inline void CovarianceMatrixFull<FloatType_, Size_>::setVariance(const sint32 idx, const FloatType_ val)
{
  constexpr auto zero = static_cast<FloatType_>(0.0);
  for (sint32 j = 0; j < Size_; ++j)
  {
    BaseSquareMatrix::at_unsafe(idx, j) = zero;
    BaseSquareMatrix::at_unsafe(j, idx) = zero;
  }
  BaseSquareMatrix::at_unsafe(idx, idx) = val;
}

} // namespace math
} // namespace tracking

#endif // E4324F70_8D26_4E47_B84E_1A44466A0FBB
