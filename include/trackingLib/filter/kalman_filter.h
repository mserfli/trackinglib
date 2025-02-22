#ifndef DA0115C7_88CB_4774_A6A3_54764AF1BF9D
#define DA0115C7_88CB_4774_A6A3_54764AF1BF9D

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"

namespace tracking
{
namespace filter
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <typename FloatType_>
class KalmanFilter
{
public:
  template <sint32 DimX_, sint32 DimQ_>
  inline static void predictCovariance(math::CovarianceMatrixFull<FloatType_, DimX_>& P,
                                       const math::SquareMatrix<FloatType_, DimX_>&   A,
                                       const math::Matrix<FloatType_, DimX_, DimQ_>&  G,
                                       const math::DiagonalMatrix<FloatType_, DimQ_>& Q);

  // prediction for UD factored covariance
  template <sint32 DimX_, sint32 DimQ_>
  inline static void predictCovariance(math::CovarianceMatrixFactored<FloatType_, DimX_>& P,
                                       const math::SquareMatrix<FloatType_, DimX_>&       A,
                                       const math::Matrix<FloatType_, DimX_, DimQ_>&      G,
                                       const math::DiagonalMatrix<FloatType_, DimQ_>&     Q);
};

} // namespace filter
} // namespace tracking

#endif // DA0115C7_88CB_4774_A6A3_54764AF1BF9D
