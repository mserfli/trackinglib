#ifndef CD00E333_97EF_4391_B880_C543B45E2D3F
#define CD00E333_97EF_4391_B880_C543B45E2D3F

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/matrix.h"
#include "math/linalg/square_matrix.h"

namespace tracking
{
namespace filter
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <typename FloatType_>
class InformationFilter
{
public:
  template <sint32 DimX_, sint32 DimQ_>
  static void predictCovariance(math::CovarianceMatrixFull<FloatType_, DimX_>& Y,
                                const math::SquareMatrix<FloatType_, DimX_>&   A,
                                const math::Matrix<FloatType_, DimX_, DimQ_>&  G,
                                const math::DiagonalMatrix<FloatType_, DimQ_>& Q);

  // prediction for UD factored covariance
  template <sint32 DimX_, sint32 DimQ_>
  static void predictCovariance(math::CovarianceMatrixFactored<FloatType_, DimX_>& Y,
                                const math::SquareMatrix<FloatType_, DimX_>&       A,
                                const math::Matrix<FloatType_, DimX_, DimQ_>&      G,
                                const math::DiagonalMatrix<FloatType_, DimQ_>&     Q);
};

} // namespace filter
} // namespace tracking

#endif // CD00E333_97EF_4391_B880_C543B45E2D3F
