#ifndef covariance_matrix_factored_h
#define covariance_matrix_factored_h

#include "base/matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size>
class CovarianceMatrixFactored
{
public:
  static constexpr sint32 rows = Size;
  using const_type             = std::add_const<FloatType>;
  using type                   = std::remove_const<FloatType>;
  using self                   = CovarianceMatrixFactored<type, rows>;
  using FullMatrix             = SquareMatrix<type, rows>;
};

} // namespace base
} // namespace tracking

#endif /* covariance_matrix_factored_h */
