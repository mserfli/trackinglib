#ifndef D68AD8A7_E600_4155_9580_2BA4AE63E9D4
#define D68AD8A7_E600_4155_9580_2BA4AE63E9D4

#include "math/linalg/covariance_matrix_factored.h"

#include "math/linalg/covariance_matrix_full.hpp" // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.hpp"        // IWYU pragma: keep
#include "math/linalg/matrix.hpp"                 // IWYU pragma: keep
#include "math/linalg/matrix_column_view.hpp"     // IWYU pragma: keep
#include "math/linalg/matrix_row_view.hpp"        // IWYU pragma: keep
#include "math/linalg/modified_gram_schmidt.hpp"  // IWYU pragma: keep
#include "math/linalg/rank1_update.hpp"           // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"          // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp"      // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename FloatType_, sint32 Size_>
CovarianceMatrixFactored<FloatType_, Size_>::CovarianceMatrixFactored(const TriangularMatrix<FloatType_, Size_, false, true>& u,
                                                                      const DiagonalMatrix<FloatType_, Size_>&                d,
                                                                      const bool isInverse)
    : _u{u}
    , _d{d}
    , _isInverse{isInverse}
{
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType_, sint32 Size_>
auto CovarianceMatrixFactored<FloatType_, Size_>::FromList(const std::initializer_list<std::initializer_list<value_type>>& u,
                                                           const std::initializer_list<value_type>&                        d,
                                                           const bool isInverse) -> CovarianceMatrixFactored
{
  CovarianceMatrixFactored cov{
      TriangularMatrix<FloatType_, Size_, false, true>::FromList(u), DiagonalMatrix<FloatType_, Size_>::FromList(d), isInverse};
  return cov;
}

template <typename FloatType_, sint32 Size_>
auto CovarianceMatrixFactored<FloatType_, Size_>::FromList(const std::initializer_list<std::initializer_list<value_type>>& list,
                                                           const bool isInverse) -> CovarianceMatrixFactored
{
  auto other  = compose_type::SquareMatrix::FromList(list);
  auto retVal = other.decomposeUDUT();
  assert(retVal.has_value());

  auto [u, d] = retVal.value_or(std::make_pair(TriangularMatrix<FloatType_, Size_, false, true>::Identity(),
                                               DiagonalMatrix<FloatType_, Size_>::Identity()));
  return CovarianceMatrixFactored{std::move(u), std::move(d), isInverse};
}


template <typename FloatType_, sint32 Size_>
auto CovarianceMatrixFactored<FloatType_, Size_>::Identity() -> CovarianceMatrixFactored
{
  CovarianceMatrixFactored cov{TriangularMatrix<FloatType_, Size_, false, true>::Identity(),
                               DiagonalMatrix<FloatType_, Size_>::Identity()};
  return cov;
}

template <typename FloatType_, sint32 Size_>
void CovarianceMatrixFactored<FloatType_, Size_>::setIdentity()
{
  _u.setIdentity();
  _d.setIdentity();
}

template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactored<FloatType_, Size_>::operator()(sint32 row,
                                                                    sint32 col) const -> tl::expected<value_type, Errors>
{
  if (!(row >= 0 && row < dim))
  {
    return tl::unexpected<Errors>{Errors::invalid_access_row};
  }
  if (!(col >= 0 && col < dim))
  {
    return tl::unexpected<Errors>{Errors::invalid_access_col};
  }

  return at_unsafe(row, col);
}

template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactored<FloatType_, Size_>::at_unsafe(sint32 row, sint32 col) const -> FloatType_
{
  // cov(row,col) == cov(col,row), so we swap row and col if row > col
  if (row > col)
  {
    std::swap(row, col);
  }

  FloatType_ result{};
  if (_isInverse)
  { // calc row, col element of _u.transpose() * _d * _u
    // calc relevant elements of d*u to be lhs multiplied with uT(col::, col)==u(col, col::)
    Vector<FloatType_, Size_> du{};
    for (auto i = 0; i <= row; ++i)
    {
      du.at_unsafe(i) = _d.at_unsafe(i) * _u.at_unsafe(i, col);
    }
    MatrixColumnView<FloatType_, Size_, 1, true>     duView{du, 0, 0, row};
    MatrixColumnView<FloatType_, Size_, Size_, true> uTView{_u, row, 0, row};
    result = uTView * duView;
  }
  else
  { // calc row, col element of _u * _d * _u.transpose()
    // calc relevant elements of u*d to be rhs multiplied with uT(col::, col)==u(col, col::)
    Vector<FloatType_, Size_> ud{};
    for (auto i = col; i < Size_; ++i)
    {
      ud.at_unsafe(i) = _u.at_unsafe(row, i) * _d.at_unsafe(i);
    }
    MatrixColumnView<FloatType_, Size_, 1, true>  udView{ud, 0, col, Size_ - 1};
    MatrixRowView<FloatType_, Size_, Size_, true> uTView{_u, col, col, Size_ - 1};
    result = uTView * udView; // calc the scalar product of ud*uT on relevant elements
  }
  return result;
}

template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactored<FloatType_, Size_>::operator()() const -> compose_type
{
  if (_isInverse)
  {
    auto s = typename compose_type::SquareMatrix{_u.transpose() * _d * _u};
    // symmetrize
    s += s.transpose();
    s *= static_cast<FloatType_>(0.5);
    return compose_type{std::move(s), _isInverse};
  }
  else
  {
    auto s = typename compose_type::SquareMatrix{_u * _d * _u.transpose()};
    // symmetrize
    s += s.transpose();
    s *= static_cast<FloatType_>(0.5);
    return compose_type{std::move(s), _isInverse};
  }
}

template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactored<FloatType_, Size_>::inverse() const -> tl::expected<CovarianceMatrixFactored, Errors>
{
  return CovarianceMatrixFactored{std::move(_u.inverse()), std::move(_d.inverse()), !_isInverse};
}

template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactored<FloatType_, Size_>::isInverse() const -> bool
{
  return _isInverse;
}

template <typename FloatType_, sint32 Size_>
inline void CovarianceMatrixFactored<FloatType_, Size_>::apaT(const SquareMatrix<FloatType_, Size_, true>& A)
{
  // TODO(matthias): Grewal, p. 260 -> inplace product Phi*U
  if (_isInverse)
  {
    const auto invPhi  = A.inverse();
    auto       invPhiU = SquareMatrix<FloatType_, Size_, true>{invPhi.transpose() * _u.transpose()};
    // cov = U'DU
    // _u is not read, but fully overwritten; invPhiU is read and updated
    math::ModifiedGramSchmidt<FloatType_, Size_>::run(_u, _d, std::move(invPhiU));
    // cov = UDU'
    _isInverse = false;
  }
  else
  {
    auto PhiU = SquareMatrix<FloatType_, Size_, true>{A * _u};
    // _u is not read, but fully overwritten; PhiU is read and updated
    math::ModifiedGramSchmidt<FloatType_, Size_>::run(_u, _d, std::move(PhiU));
  }
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType_, sint32 Size_>
inline auto CovarianceMatrixFactored<FloatType_, Size_>::apaT(const SquareMatrix<FloatType_, Size_, true>& A) const
    -> CovarianceMatrixFactored
{
  CovarianceMatrixFactored cov{*this};
  cov.apaT(A);
  return cov;
}

template <typename FloatType_, sint32 Size_>
template <sint32 SizeQ_>
inline void CovarianceMatrixFactored<FloatType_, Size_>::thornton(const SquareMatrix<FloatType_, Size_, true>&   Phi,
                                                                  const Matrix<FloatType_, Size_, SizeQ_, true>& G,
                                                                  const DiagonalMatrix<FloatType_, SizeQ_>&      Q)
{
  math::ModifiedGramSchmidt<FloatType_, Size_>::run(_u, _d, Phi, G, Q);
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType_, sint32 Size_>
inline void CovarianceMatrixFactored<FloatType_, Size_>::rank1Update(const FloatType_ c, const Vector<FloatType_, Size_>& x)
{
  if (_isInverse)
  {
    // TODO(matthias): find a solution without transposing and copying the matrix
    typename TriangularMatrix<FloatType_, Size_, false, true>::transpose_type l{_u.transpose()};
    math::Rank1Update<FloatType_, Size_, false>::run(l, _d, c, x);
    _u = std::move(l.transpose());
  }
  else
  {
    math::Rank1Update<FloatType_, Size_, true>::run(_u, _d, c, x);
  }
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType_, sint32 Size_>
inline void CovarianceMatrixFactored<FloatType_, Size_>::setVariance(const sint32 idx, const FloatType_ val)
{
  auto A                = SquareMatrix<FloatType_, Size_, true>::Identity();
  A.at_unsafe(idx, idx) = static_cast<FloatType_>(0.0);
  apaT(A);
  setDiagonal(idx, val);
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType_, sint32 Size_>
template <sint32 SrcSize_, sint32 SrcCount_>
inline void CovarianceMatrixFactored<FloatType_, Size_>::fill(const CovarianceMatrixFactored<FloatType_, SrcSize_>& other)
{
  _u.template setBlock<SrcSize_, SrcCount_, 0, 0, 0, 0>(other._u);
  _d.template setBlock<SrcSize_, SrcCount_, 0, 0>(other._d);
  _isInverse = other._isInverse;
  assert(_u.isUnitUpperTriangular() && "Bad triangular matrix not fullfilling the constraint IsUnitUpperTriangular");
  assert(_d.isPositiveDefinite() && "Bad diagonal matrix not fullfilling the constraint isPositiveDefinite");
}

template <typename FloatType_, sint32 Size_>
inline void CovarianceMatrixFactored<FloatType_, Size_>::setDiagonal(const sint32 idx, const FloatType_ val)
{
  assert(val > static_cast<FloatType_>(0.0) && "Expected variance value greater than 0.0");
  _d.at_unsafe(idx) = val;
}
} // namespace math
} // namespace tracking

#endif // D68AD8A7_E600_4155_9580_2BA4AE63E9D4
