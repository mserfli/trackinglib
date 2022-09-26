#ifndef DD2B2494_7486_42BA_84E2_32308E26DBBC
#define DD2B2494_7486_42BA_84E2_32308E26DBBC

#include "base/square_matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size>
class CovarianceMatrixFull: public SquareMatrix<FloatType, Size>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using SquareMatrix<FloatType, Size>::SquareMatrix;

  /// \brief Construct a new Covariance Matrix Full< Float Type,  Size> object
  /// \param[in] other A base class object
  CovarianceMatrixFull<FloatType, Size>(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  auto inverse() const -> CovarianceMatrixFull<FloatType, Size>;
};

template <typename FloatType, sint32 Size>
CovarianceMatrixFull<FloatType, Size>::CovarianceMatrixFull(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{other}
{
}

template <typename FloatType, sint32 Size>
inline auto CovarianceMatrixFull<FloatType, Size>::inverse() const -> CovarianceMatrixFull<FloatType, Size>
{
  CovarianceMatrixFull<FloatType, Size>   inv{};
  TriangularMatrix<FloatType, Size, true> L{};

  auto isOk = this->decomposeLLT(L);
  assert(isOk && "covariance not positive definite");

  // L*(L'*Ainv) = eye(n,n)
  // L*u = eye(n,n)  -> solve for u using forward substitution on each column vector of eye(n,n)
  auto u = std::move(L.solve(SquareMatrix<FloatType, Size>::Identity()));
  // L'*Ainv = u     -> solve for Ainv using backward substitution
  inv = std::move(L.transpose().solve(u));

  return inv;
}

} // namespace base
} // namespace tracking

#endif // DD2B2494_7486_42BA_84E2_32308E26DBBC
