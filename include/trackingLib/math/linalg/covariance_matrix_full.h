#ifndef DD2B2494_7486_42BA_84E2_32308E26DBBC
#define DD2B2494_7486_42BA_84E2_32308E26DBBC

#include "base/first_include.h"
#include "math/linalg/contracts/covariance_matrix_intf.h"
#include "math/linalg/square_matrix.hpp"

namespace tracking
{
namespace math
{

// TODO(matthias): add contract for apaT functions
template <typename FloatType, sint32 Size>
class CovarianceMatrixFull
    : public SquareMatrix<FloatType, Size>
    , public contract::CovarianceMatrixIntf<CovarianceMatrixFull<FloatType, Size>>
{
public:
  using value_type   = FloatType;
  using compose_type = CovarianceMatrixFull;

  //  rule of 5 declarations
  CovarianceMatrixFull()                                = default;
  CovarianceMatrixFull(const CovarianceMatrixFull&)     = default;
  CovarianceMatrixFull(CovarianceMatrixFull&&) noexcept = default;
  auto operator=(const CovarianceMatrixFull&) -> CovarianceMatrixFull& = default;
  auto operator=(CovarianceMatrixFull&&) noexcept -> CovarianceMatrixFull& = default;
  virtual ~CovarianceMatrixFull()                                          = default;


  /*  CovarianceMatrixFull() = default;
    CovarianceMatrixFull(const CovarianceMatrixFull& other)
        : SquareMatrix<FloatType, Size>{other}
        , contract::CovarianceMatrixIntf<CovarianceMatrixFull<FloatType, Size>>{}
        , _isInverse{other._isInverse}
    {
    }
    CovarianceMatrixFull(CovarianceMatrixFull&& other) noexcept
        : SquareMatrix<FloatType, Size>{std::move(other)}
        , contract::CovarianceMatrixIntf<CovarianceMatrixFull<FloatType, Size>>{}
        , _isInverse{other._isInverse}
    {
      // make other unusable using placement new operator
      new (&other) CovarianceMatrixFull<FloatType, Size>{};
    }
    auto operator=(const CovarianceMatrixFull& other) -> CovarianceMatrixFull&
    {
      if (this != &other)
      {
        SquareMatrix<FloatType, Size>::operator=(other);
        _isInverse                             = other._isInverse;
      }
      return *this;
    };
    auto operator=(CovarianceMatrixFull&& other) noexcept -> CovarianceMatrixFull&
    {
      if (this != &other)
      {
        SquareMatrix<FloatType, Size>::operator=(std::move(other));
        // init remaining data
        _isInverse = other._isInverse;

        // make other unusable using placement new operator
        new (&other) CovarianceMatrixFull<FloatType, Size>{};
      }
      return *this;
    }
    ~CovarianceMatrixFull() = default; */

  /// \brief Construct a new Covariance Matrix Full< Float Type,  Size> object
  /// \param[in] other A base class object
  /// \param[in,out] isInverse
  explicit CovarianceMatrixFull(const SquareMatrix<FloatType, Size>& other, const bool isInverse = false);

  /// \brief Construct an Identity matrix
  /// \return CovarianceMatrixFull
  static auto Identity() -> CovarianceMatrixFull;

  /// \brief Set Identity covariance
  void setIdentity();

  /// \brief Access operator to the covariance value at (row, col)
  /// \param[in,out] row  The specified row
  /// \param[in,out] col  The specified column
  /// \return FloatType
  auto operator()(sint32 row, sint32 col) const -> FloatType;

  /// \brief Creates the "composed" covariance, although no composition is needed
  /// \return CovarianceMatrixFull
  auto operator()() const -> CovarianceMatrixFull
  {
    assert(0 && "avoid calling this function.");
    return *this;
  }

  /// \brief Calculates the inverse based on Cholesky decomposition
  /// \return CovarianceMatrixFull
  auto inverse() const -> CovarianceMatrixFull;

  /// \brief
  /// \return true
  /// \return false
  auto isInverse() const -> bool;

  /// \brief Calculate A*P*A' inplace
  /// \param[in] A
  void apaT(const SquareMatrix<FloatType, Size>& A);

  /// \brief Calculate A*P*A'
  /// \param[in] A
  auto apaT(const SquareMatrix<FloatType, Size>& A) const -> CovarianceMatrixFull;

  /// \brief Set the variance at (idx,idx) and clears any correlations
  /// \param[in] idx  Index in diagonal matrix
  /// \param[in] val  The value to be set
  void setVariance(const sint32 idx, const FloatType val);

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  bool _isInverse{false};
};

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
  inv = CovarianceMatrixFull(L.transpose().solve(u), !_isInverse);

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

#endif // DD2B2494_7486_42BA_84E2_32308E26DBBC
