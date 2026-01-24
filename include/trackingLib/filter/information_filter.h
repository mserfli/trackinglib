#ifndef CD00E333_97EF_4391_B880_C543B45E2D3F
#define CD00E333_97EF_4391_B880_C543B45E2D3F

#include "base/first_include.h"                                  // IWYU pragma: keep
#include "math/linalg/contracts/covariance_matrix_policy_intf.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_policies.h"              // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/matrix.h"
#include "math/linalg/square_matrix.h"

namespace tracking
{
namespace filter
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <typename CovarianceMatrixPolicy_>
class InformationFilter: public math::contract::CovarianceMatrixPolicyIntf<CovarianceMatrixPolicy_>
{
public:
  using value_type = typename CovarianceMatrixPolicy_::value_type;
  template <sint32 DimX_>
  using CovarianceMatrixType = typename CovarianceMatrixPolicy_::template Instantiate<DimX_>;

  template <sint32 DimX_, sint32 DimQ_>
  static void predictCovariance(CovarianceMatrixType<DimX_>&                   Y,
                                const math::SquareMatrix<value_type, DimX_>&   A,
                                const math::Matrix<value_type, DimX_, DimQ_>&  G,
                                const math::DiagonalMatrix<value_type, DimQ_>& Q);
};

} // namespace filter
} // namespace tracking

#endif // CD00E333_97EF_4391_B880_C543B45E2D3F
