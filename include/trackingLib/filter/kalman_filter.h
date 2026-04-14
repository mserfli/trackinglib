#ifndef DA0115C7_88CB_4774_A6A3_54764AF1BF9D
#define DA0115C7_88CB_4774_A6A3_54764AF1BF9D

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
class KalmanFilter: public math::contract::CovarianceMatrixPolicyIntf<CovarianceMatrixPolicy_>
{
public:
  using value_type = typename CovarianceMatrixPolicy_::value_type;
  template <sint32 DimX_>
  using CovarianceMatrixType = typename CovarianceMatrixPolicy_::template Instantiate<DimX_>;

  template <sint32 DimX_, sint32 DimQ_>
  inline static void predictCovariance(CovarianceMatrixType<DimX_>&                   P,
                                       const math::SquareMatrix<value_type, DimX_>&   A,
                                       const math::Matrix<value_type, DimX_, DimQ_>&  G,
                                       const math::DiagonalMatrix<value_type, DimQ_>& Q);
};

} // namespace filter
} // namespace tracking

#endif // DA0115C7_88CB_4774_A6A3_54764AF1BF9D
