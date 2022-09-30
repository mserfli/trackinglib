#ifndef DD2B2494_7486_42BA_84E2_32308E26DBBC
#define DD2B2494_7486_42BA_84E2_32308E26DBBC

#include "base/first_include.h"
#include "math/linalg/square_matrix.h"

namespace tracking
{
namespace math
{

// TODO(matthias): add interface contract
template <typename FloatType, sint32 Size>
class CovarianceMatrixFull: public SquareMatrix<FloatType, Size>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using SquareMatrix<FloatType, Size>::SquareMatrix;

  /// \brief Construct a new Covariance Matrix Full< Float Type,  Size> object
  /// \param[in] other A base class object
  CovarianceMatrixFull<FloatType, Size>(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Calculates the inverse based on Cholesky decomposition
  /// \return CovarianceMatrixFull<FloatType, Size>
  //auto inverse() const -> CovarianceMatrixFull<FloatType, Size> { return SquareMatrix<FloatType, Size>::inverse(); }
};

template <typename FloatType, sint32 Size>
CovarianceMatrixFull<FloatType, Size>::CovarianceMatrixFull(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{other}
{
}

} // namespace math
} // namespace tracking

#endif // DD2B2494_7486_42BA_84E2_32308E26DBBC
