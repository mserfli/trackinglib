#ifndef F9044FD7_A3A8_43F4_BDD6_F43011384722
#define F9044FD7_A3A8_43F4_BDD6_F43011384722

#include "base/first_include.h"
#include "base/atomic_types.h"
#include "math/linalg/agee_turner_rank1_update.hpp"
#include "math/linalg/contracts/covariance_matrix_intf.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/matrix.h"
#include "math/linalg/modified_gram_schmidt.hpp"
#include "math/linalg/square_matrix.h"
#include "math/linalg/triangular_matrix.h"

namespace tracking
{
namespace math
{
// TODO(matthias): add contract for apaT functions, fill, ...
template <typename FloatType, sint32 Size>
class CovarianceMatrixFactored: public contract::CovarianceMatrixIntf<CovarianceMatrixFactored<FloatType, Size>>
{
public:
  using value_type   = FloatType;
  using compose_type = CovarianceMatrixFull<FloatType, Size>;

  // rule of 5 declarations
  CovarianceMatrixFactored()                                    = default;
  CovarianceMatrixFactored(const CovarianceMatrixFactored&)     = default;
  CovarianceMatrixFactored(CovarianceMatrixFactored&&) noexcept = default;
  auto operator=(const CovarianceMatrixFactored&) -> CovarianceMatrixFactored& = default;
  auto operator=(CovarianceMatrixFactored&&) noexcept -> CovarianceMatrixFactored& = default;

  explicit CovarianceMatrixFactored(const SquareMatrix<FloatType, Size>& other, const bool isInverse = false);
  explicit CovarianceMatrixFactored(const TriangularMatrix<FloatType, Size, false>& u,
                                    const DiagonalMatrix<FloatType, Size>&          d,
                                    const bool                                      isInverse = false);

  /// \brief Construct an Identity matrix
  /// \return CovarianceMatrixFactored
  static auto Identity() -> CovarianceMatrixFactored;

  /// \brief Set Identity covariance
  void setIdentity();

  /// \brief Access operator to the covariance value at (row, col)
  /// \param[in,out] row  The specified row
  /// \param[in,out] col  The specified column
  /// \return FloatType
  auto operator()(sint32 row, sint32 col) const -> FloatType;

  /// \brief Creates the composed covariance
  /// \return CovarianceMatrixFull<FloatType, Size>
  auto operator()() const -> CovarianceMatrixFull<FloatType, Size>;

  /// \brief Calculates the inverse
  /// \return CovarianceMatrixFactored
  auto inverse() const -> CovarianceMatrixFactored;

  /// \brief
  /// \return true
  /// \return false
  auto isInverse() const -> bool;

  /// \brief Calculate A*P*A' inplace
  /// \param[in] A
  void apaT(const SquareMatrix<FloatType, Size>& A);

  /// \brief Calculate A*P*A'
  /// \param[in] A
  auto apaT(const SquareMatrix<FloatType, Size>& A) const -> CovarianceMatrixFactored;

  /// \brief Calculate A*P*A' + G*Q*G', also known as Thornton update
  /// \tparam SizeQ
  /// \param[in,out] Phi
  /// \param[in,out] G
  /// \param[in,out] Q
  template <sint32 SizeQ>
  void thornton(const SquareMatrix<FloatType, Size>&    Phi,
                const Matrix<FloatType, Size, SizeQ>&   G,
                const DiagonalMatrix<FloatType, SizeQ>& Q);

  /// \brief Calculates P - c*x*x', also known as Agee Turner Rank-1 update
  /// \param[in] c
  /// \param[in] x
  void rank1Update(const FloatType c, const Vector<FloatType, Size>& x);

  /// \brief Set the variance at (idx,idx) and clears any correlations
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setVariance(const sint32 idx, const FloatType val);

  /// \brief Fill the covariance with first N=SrcCount rows and cols of the other covariance
  /// \tparam SrcSize   Size of the other covariance
  /// \tparam SrcCount  Count rows/cols to copy from other
  /// \param[in] other  The other matrix to copy from
  template <sint32 SrcSize, sint32 SrcCount>
  void fill(const CovarianceMatrixFactored<FloatType, SrcSize>& other);

  /// \brief Set the Diagonal matrix element to given value
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setDiagonal(const sint32 idx, const FloatType val);

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  TriangularMatrix<FloatType, Size, false> _u{};
  DiagonalMatrix<FloatType, Size>          _d{};
  bool                                     _isInverse{false};
};

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
    , _isInverse(isInverse)
{
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
    cov = _u.transpose() * _d * _u;
  }
  else
  {
    cov = _u * _d * _u.transpose();
  }
  return cov;
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFactored<FloatType, Size>::inverse() const -> CovarianceMatrixFactored
{
  return CovarianceMatrixFactored{_u.inverse(), _d.inverse(), true};
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
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFactored<FloatType, Size>::rank1Update(const FloatType c, const Vector<FloatType, Size>& x)
{
  if (_isInverse)
  {
    math::AgeeTurnerRank1Update<FloatType, Size>::run(_u, _d, c, x, true);
    // TODO(matthias): do we need to reset _isInverse???
  }
  else
  {
    math::AgeeTurnerRank1Update<FloatType, Size>::run(_u, _d, c, x, false);
  }
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFactored<FloatType, Size>::setVariance(const sint32 idx, const FloatType val)
{
  auto A      = SquareMatrix<FloatType, Size>::Identity();
  A(idx, idx) = static_cast<FloatType>(0.0);
  apaT(A);
  setDiagonal(idx, val);
}

template <typename FloatType, sint32 Size>
template <sint32 SrcSize, sint32 SrcCount>
inline void CovarianceMatrixFactored<FloatType, Size>::fill(const CovarianceMatrixFactored<FloatType, SrcSize>& other)
{
  _u.template setBlock<SrcSize, SrcCount, 0, 0, 0, 0>(other._u);
  _d.template setBlock<SrcSize, SrcCount, 0, 0>(other._d);
  _isInverse = other._isInverse;
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFactored<FloatType, Size>::setDiagonal(const sint32 idx, const FloatType val)
{
  _d[idx] = val;
}

} // namespace math
} // namespace tracking

#endif // F9044FD7_A3A8_43F4_BDD6_F43011384722
