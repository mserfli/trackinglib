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
  using value_type = FloatType;
  using compose_type = CovarianceMatrixFull;

  /// \brief Inherit Rule of 5 behavior from base class
  using SquareMatrix<FloatType, Size>::SquareMatrix;

  /// \brief Construct a new Covariance Matrix Full< Float Type,  Size> object
  /// \param[in] other A base class object
  CovarianceMatrixFull(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

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
};

template <typename FloatType, sint32 Size>
CovarianceMatrixFull<FloatType, Size>::CovarianceMatrixFull(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{other}
{
}

template <typename FloatType, sint32 Size>
auto CovarianceMatrixFull<FloatType, Size>::Identity() -> CovarianceMatrixFull
{
  CovarianceMatrixFull cov{SquareMatrix<FloatType, Size>::Identity()};
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
  return SquareMatrix<FloatType, Size>::inverse();
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFull<FloatType, Size>::apaT(const SquareMatrix<FloatType, Size>& A)
{
  this->operator=(A * this->operator*=(A.transpose()));
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFull<FloatType, Size>::apaT(const SquareMatrix<FloatType, Size>& A) const -> CovarianceMatrixFull
{
  return (A * (*this) * A.transpose());
}

template <typename FloatType, sint32 Size>
inline void CovarianceMatrixFull<FloatType, Size>::setVariance(const sint32 idx, const FloatType val)
{
  constexpr auto zero = static_cast<FloatType>(0.0);
  for(sint32 j=0;j<Size;++j)
  {
    SquareMatrix<FloatType, Size>::operator()(idx, j) = zero;
    SquareMatrix<FloatType, Size>::operator()(j, idx) = zero;
  }
  SquareMatrix<FloatType, Size>::operator()(idx, idx) = val;
}

} // namespace math
} // namespace tracking

#endif // DD2B2494_7486_42BA_84E2_32308E26DBBC
