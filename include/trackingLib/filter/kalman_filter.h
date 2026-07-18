#ifndef DA0115C7_88CB_4774_A6A3_54764AF1BF9D
#define DA0115C7_88CB_4774_A6A3_54764AF1BF9D

#include "base/first_include.h"                                  // IWYU pragma: keep
#include "filter/update_mode.h"                                  // IWYU pragma: keep
#include "math/linalg/contracts/covariance_matrix_policy_intf.h" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_policies.h"              // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/matrix.h"
#include "math/linalg/square_matrix.h"
#include "math/linalg/vector.h"

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

  /// \brief Kalman measurement update of state and covariance
  ///
  /// Applies the (extended) Kalman measurement update to the given state vector x and state
  /// covariance P using the innovation nu = z - h(x), the measurement Jacobian H = dh/dx and
  /// the measurement covariance R.
  ///
  /// Update modes:
  /// - update_mode::Block:      single DimZ-sized update with Joseph stabilized covariance;
  ///                            supports correlated R; full covariance policy only.
  /// - update_mode::Sequential: scalar row-by-row updates (rank-1 covariance modifications);
  ///                            keeps the UDU factorization intact for the factored policy.
  ///                            A correlated R is decorrelated first via R = U*D*U'
  ///                            (nu' = inv(U)*nu, H' = inv(U)*H, R' = D), making the result
  ///                            equivalent to the block update.
  ///
  /// \tparam UpdateMode_ Update mode tag (update_mode::Block or update_mode::Sequential)
  /// \tparam DimX_ State dimension
  /// \tparam DimZ_ Measurement dimension
  /// \param[in,out] x           State vector to be updated
  /// \param[in,out] P           State covariance to be updated
  /// \param[in]     innovation  Innovation nu = z - h(x) evaluated at the given state
  /// \param[in]     H           Measurement Jacobian dh/dx evaluated at the given state
  /// \param[in]     R           Measurement covariance
  /// \note Measurement rows with a non-positive innovation variance are skipped defensively.
  template <typename UpdateMode_ = update_mode::Default<CovarianceMatrixPolicy_>, sint32 DimX_, sint32 DimZ_>
  inline static void updateState(math::Vector<value_type, DimX_>&              x,
                                 CovarianceMatrixType<DimX_>&                  P,
                                 const math::Vector<value_type, DimZ_>&        innovation,
                                 const math::Matrix<value_type, DimZ_, DimX_>& H,
                                 const CovarianceMatrixType<DimZ_>&            R);
};

} // namespace filter
} // namespace tracking

#endif // DA0115C7_88CB_4774_A6A3_54764AF1BF9D
