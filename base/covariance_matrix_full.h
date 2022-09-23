#ifndef covariance_matrix_full_h
#define covariance_matrix_full_h

#include "base/matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size>
using CovarianceMatrixFull = Eigen::Matrix<FloatType, Size, Size>;
#if 0
class CovarianceMatrixFull: public SquareMatrix<FloatType, Size>
{
public:
  static constexpr sint32 rows = Size;
  using const_type             = std::add_const<FloatType>;
  using type                   = std::remove_const<FloatType>;
  using self                   = CovarianceMatrixFull<type, rows>;
  using FullMatrix             = SquareMatrix<type, rows>;
};
#endif
} // namespace base
} // namespace tracking

#endif /* covariance_matrix_full_h */
