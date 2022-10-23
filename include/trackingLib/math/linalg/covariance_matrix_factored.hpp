#ifndef D68AD8A7_E600_4155_9580_2BA4AE63E9D4
#define D68AD8A7_E600_4155_9580_2BA4AE63E9D4

#include "math/linalg/covariance_matrix_factored.h"

#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.hpp"
#include "math/linalg/matrix.h"
#include "math/linalg/modified_gram_schmidt.hpp"
#include "math/linalg/rank1_update.hpp"
#include "math/linalg/square_matrix.hpp"
#include "math/linalg/triangular_matrix.hpp"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
CovarianceMatrixFactored<FloatType, Size>::CovarianceMatrixFactored(const SquareMatrix<FloatType, Size>& other,
                                                                    const bool                           isInverse)
{
  other.decomposeUDUT(_u, _d);
  _isInverse = isInverse;
}

template <typename FloatType, sint32 Size>
CovarianceMatrixFactored<FloatType, Size>::CovarianceMatrixFactored(const TriangularMatrix<FloatType, Size, false>& u,
                                                                    const DiagonalMatrix<FloatType, Size>&          d,
                                                                    const bool                                      isInverse)
    : _u{u}
    , _d{d}
    , _isInverse{isInverse}
{
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType, sint32 Size>
auto CovarianceMatrixFactored<FloatType, Size>::Identity() -> CovarianceMatrixFactored
{
  CovarianceMatrixFactored cov{TriangularMatrix<FloatType, Size, false>::Identity(), DiagonalMatrix<FloatType, Size>::Identity()};
  return cov;
}

template <typename FloatType, sint32 Size>
void CovarianceMatrixFactored<FloatType, Size>::setIdentity()
{
  _u.setIdentity();
  _d.setIdentity();
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFactored<FloatType, Size>::operator()(sint32 row, sint32 col) const -> FloatType
{
  // TODO(matthias): optimize by calculating only the requested covariance
  // element
  auto tmp = this->operator()();
  return tmp(row, col);
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFactored<FloatType, Size>::operator()() const -> CovarianceMatrixFull<FloatType, Size>
{
  CovarianceMatrixFull<FloatType, Size> cov{};
  if (_isInverse)
  {
    cov = CovarianceMatrixFull<FloatType, Size>(_u.transpose() * _d * _u, _isInverse);
  }
  else
  {
    cov = CovarianceMatrixFull<FloatType, Size>(_u * _d * _u.transpose(), _isInverse);
  }
  return cov;
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFactored<FloatType, Size>::inverse() const -> CovarianceMatrixFactored
{
  return CovarianceMatrixFactored{_u.inverse(), _d.inverse(), !_isInverse};
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFactored<FloatType, Size>::isInverse() const -> bool
{
  return _isInverse;
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFactored<FloatType, Size>::apaT(const SquareMatrix<FloatType, Size>& A)
{
  if (_isInverse)
  {
    math::ModifiedGramSchmidt<FloatType, Size>::run(_u, _d, A, true);
    _isInverse = false; // reset internal isInverse flag
  }
  else
  {
    math::ModifiedGramSchmidt<FloatType, Size>::run(_u, _d, A, false);
  }
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFactored<FloatType, Size>::apaT(const SquareMatrix<FloatType, Size>& A) const
    -> CovarianceMatrixFactored
{
  CovarianceMatrixFactored cov{*this};
  cov.apaT(A);
  return cov;
}

template <typename FloatType, sint32 Size>
template <sint32 SizeQ>
inline void CovarianceMatrixFactored<FloatType, Size>::thornton(const SquareMatrix<FloatType, Size>&    Phi,
                                                                const Matrix<FloatType, Size, SizeQ>&   G,
                                                                const DiagonalMatrix<FloatType, SizeQ>& Q)
{
  math::ModifiedGramSchmidt<FloatType, Size>::run(_u, _d, Phi, G, Q);
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFactored<FloatType, Size>::rank1Update(const FloatType c, const Vector<FloatType, Size>& x)
{
  if (_isInverse)
  {
    // TODO(matthias): find a solution without transposing and copying the matrix
    TriangularMatrix<FloatType, Size, true> l = _u.transpose();
    math::Rank1Update<FloatType, Size>::run(l, _d, c, x);
    _u = l.transpose();
  }
  else
  {
    math::Rank1Update<FloatType, Size>::run(_u, _d, c, x);
  }

  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFactored<FloatType, Size>::setVariance(const sint32 idx, const FloatType val)
{
  auto A      = SquareMatrix<FloatType, Size>::Identity();
  A(idx, idx) = static_cast<FloatType>(0.0);
  apaT(A);
  setDiagonal(idx, val);
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType, sint32 Size>
template <sint32 SrcSize, sint32 SrcCount>
inline void CovarianceMatrixFactored<FloatType, Size>::fill(const CovarianceMatrixFactored<FloatType, SrcSize>& other)
{
  _u.template setBlock<SrcSize, SrcCount, 0, 0, 0, 0>(other._u);
  _d.template setBlock<SrcSize, SrcCount, 0, 0>(other._d);
  _isInverse = other._isInverse;
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFactored<FloatType, Size>::setDiagonal(const sint32 idx, const FloatType val)
{
  assert(val > static_cast<FloatType>(0.0) && "Expected variance value greater than 0.0");
  _d[idx] = val;
}

} // namespace math
} // namespace tracking

#endif // D68AD8A7_E600_4155_9580_2BA4AE63E9D4
