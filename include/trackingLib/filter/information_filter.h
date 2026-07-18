#ifndef CD00E333_97EF_4391_B880_C543B45E2D3F
#define CD00E333_97EF_4391_B880_C543B45E2D3F

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

  /// \brief Information measurement update of information vector and information matrix
  ///
  /// Applies the purely additive information-form measurement update
  ///     y += H' * inv(R) * z ;  Y += H' * inv(R) * H
  /// where z is the effective measurement of the linearized observation model
  ///     z = nu + H * x  with  nu = z_meas - h(x)
  /// (for linear observation models this equals the raw measurement).
  ///
  /// The measurement information is additive and order-independent, hence update_mode::Block
  /// and update_mode::Sequential produce identical results:
  /// - full policy:     single matrix-form addition (supports correlated R)
  /// - factored policy: DimZ rank-1 additions keeping the UDU factorization intact.
  ///                    A correlated R is decorrelated first via R = U*D*U'
  ///                    (z' = inv(U)*z, H' = inv(U)*H, R' = D).
  ///
  /// \tparam UpdateMode_ Update mode tag (update_mode::Block or update_mode::Sequential)
  /// \tparam DimX_ State dimension
  /// \tparam DimZ_ Measurement dimension
  /// \param[in,out] y  Information vector (y = Y*x) to be updated
  /// \param[in,out] Y  Information matrix (Y = inv(P)) to be updated
  /// \param[in]     z  Effective measurement z = nu + H*x of the linearized observation model
  /// \param[in]     H  Measurement Jacobian dh/dx evaluated at the state estimate
  /// \param[in]     R  Measurement covariance
  /// \note If R is not invertible (full policy) the update is skipped; measurement rows with a
  ///       non-positive variance are skipped defensively (factored policy).
  template <typename UpdateMode_ = update_mode::Default<CovarianceMatrixPolicy_>, sint32 DimX_, sint32 DimZ_>
  inline static void updateState(math::Vector<value_type, DimX_>&              y,
                                 CovarianceMatrixType<DimX_>&                  Y,
                                 const math::Vector<value_type, DimZ_>&        z,
                                 const math::Matrix<value_type, DimZ_, DimX_>& H,
                                 const CovarianceMatrixType<DimZ_>&            R);
};

} // namespace filter
} // namespace tracking

#endif // CD00E333_97EF_4391_B880_C543B45E2D3F
