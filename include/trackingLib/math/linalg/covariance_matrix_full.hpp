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

template <typename ValueType_, sint32 Size_>
auto CovarianceMatrixFull<ValueType_, Size_>::FromDiagonal(const DiagonalMatrix<ValueType_, Size_>& diag) -> CovarianceMatrixFull
{
  assert(diag.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
  return CovarianceMatrixFull{diag};
}

template <typename ValueType_, sint32 Size_>
inline auto CovarianceMatrixFull<ValueType_, Size_>::inverse() const -> tl::expected<CovarianceMatrixFull, Errors>
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
    math::SquareMatrix<ValueType_, Size_, true> cov{L.transpose().solve(u)};
    cov.symmetrize();
    return CovarianceMatrixFull{std::move(cov)};
  }
  return tl::unexpected<Errors>{retVal.error()};
}

template <typename ValueType_, sint32 Size_>
template <bool IsRowMajor_>
inline void CovarianceMatrixFull<ValueType_, Size_>::apaT(const tracking::math::SquareMatrix<ValueType_, Size_, IsRowMajor_>& A)
{
  assert(this->isSymmetric() && "Covariance currently not symmetric");
  // Stage 1: AP = A*P in full - every row of AP is needed to compute any output pair below - O(n^3)
  const auto AP = A * (*this);

  // Stage 2: only the upper triangle of AP*A^T is needed, mirrored into the lower triangle - O(n^3/2)
  BaseSquareMatrix cov{};
  for (sint32 i = 0; i < Size_; ++i)
  {
    for (sint32 j = i; j < Size_; ++j)
    {
      ValueType_ element = static_cast<ValueType_>(0);
      for (sint32 k = 0; k < Size_; ++k)
      {
        element += AP.at_unsafe(i, k) * A.at_unsafe(j, k);
      }
      // construct symmetric covariance matrix by filling both upper and lower triangle
      cov.at_unsafe(i, j) = element;
      cov.at_unsafe(j, i) = element;
    }
  }
  *this = CovarianceMatrixFull{std::move(cov)};
}

template <typename ValueType_, sint32 Size_>
template <bool IsRowMajor_>
inline auto CovarianceMatrixFull<ValueType_, Size_>::apaT(
    const tracking::math::SquareMatrix<ValueType_, Size_, IsRowMajor_>& A) const -> CovarianceMatrixFull
{
  auto copy(*this);
  copy.apaT(A);
  return copy;
}

template <typename ValueType_, sint32 Size_>
inline void CovarianceMatrixFull<ValueType_, Size_>::rank1Update(const ValueType_ c, const Vector<ValueType_, Size_>& x)
{
  assert(this->isSymmetric() && "Covariance currently not symmetric");
  // only the upper triangle of c*x*x' is accumulated; the lower triangle is mirrored to keep
  // the covariance exactly symmetric
  for (sint32 i = 0; i < Size_; ++i)
  {
    for (sint32 j = i; j < Size_; ++j)
    {
      BaseSquareMatrix::at_unsafe(i, j) += c * x.at_unsafe(i) * x.at_unsafe(j);
      BaseSquareMatrix::at_unsafe(j, i) = BaseSquareMatrix::at_unsafe(i, j);
    }
  }
}

template <typename ValueType_, sint32 Size_>
inline void CovarianceMatrixFull<ValueType_, Size_>::setVariance(const sint32 idx, const ValueType_ val)
{
  constexpr auto zero = static_cast<ValueType_>(0.0);
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
