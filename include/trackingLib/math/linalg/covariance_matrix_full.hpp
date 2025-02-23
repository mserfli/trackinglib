#ifndef E4324F70_8D26_4E47_B84E_1A44466A0FBB
#define E4324F70_8D26_4E47_B84E_1A44466A0FBB

#include "math/linalg/covariance_matrix_full.h"

#include "math/linalg/errors.h"
#include "math/linalg/square_matrix.h"
#include "math/linalg/square_matrix.hpp"     // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFull<FloatType_, Size_>::inverse() const -> tl::expected<CovarianceMatrixFull, Errors>
{
  const auto retVal = SquareMatrix::decomposeLLT();
  if (retVal.has_value())
  {
    const auto& L = *retVal;
    // L*(L'*Ainv) = eye(n,n)
    // L*u = eye(n,n)  -> solve for u using forward substitution on each column vector of eye(n,n)
    const auto u = L.solve(CovarianceMatrixFull::Identity());
    // L'*Ainv = u     -> solve for Ainv using backward substitution
    auto s = L.transpose().solve(u);

    // symmetrize
    s += s.transpose();
    s *= static_cast<FloatType_>(0.5);
    return CovarianceMatrixFull{SquareMatrix{std::move(s)}, !_isInverse};
  }
  return tl::unexpected<Errors>{retVal.error()};
}

template <typename FloatType_, sint32 Size_>
template <bool IsRowMajor_>
inline void CovarianceMatrixFull<FloatType_, Size_>::apaT(const tracking::math::SquareMatrix<FloatType_, Size_, IsRowMajor_>& A)
{
  assert(this->isSymmetric() && "Covariance currently not symmetric");
  // TODO(matthias): optimization - calculate only the upper triangle part of P and fill lower triangle part
  if (_isInverse)
  {
    const auto pa  = this->operator*(A);
    auto       res = A.transpose().operator*(pa);
    // symmetrize
    res += res.transpose();
    res *= static_cast<FloatType_>(0.5);
    *this = CovarianceMatrixFull{SquareMatrix{std::move(res)}, _isInverse};
  }
  else
  {
    const auto paT = this->operator*(A.transpose());
    auto       res = A.operator*(paT);
    // symmetrize
    res += res.transpose();
    res *= static_cast<FloatType_>(0.5);
    *this = CovarianceMatrixFull{SquareMatrix{std::move(res)}, _isInverse};
  }
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
    SquareMatrix::at_unsafe(idx, j) = zero;
    SquareMatrix::at_unsafe(j, idx) = zero;
  }
  SquareMatrix::at_unsafe(idx, idx) = val;
}

} // namespace math
} // namespace tracking

#endif // E4324F70_8D26_4E47_B84E_1A44466A0FBB
