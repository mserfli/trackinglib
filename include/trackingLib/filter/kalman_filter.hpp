#ifndef D7AF2A2A_FB74_4C03_A85F_5AC092A11582
#define D7AF2A2A_FB74_4C03_A85F_5AC092A11582

#include "filter/kalman_filter.h"

#include "filter/measurement_decorrelation.hpp"
#include "math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/matrix.hpp"                     // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"              // IWYU pragma: keep
#include "math/linalg/vector.hpp"                     // IWYU pragma: keep
#include <type_traits>

namespace tracking
{
namespace filter
{

template <typename CovarianceMatrixPolicy_>
template <sint32 DimX_, sint32 DimQ_>
inline void KalmanFilter<CovarianceMatrixPolicy_>::predictCovariance(CovarianceMatrixType<DimX_>&                   P,
                                                                     const math::SquareMatrix<value_type, DimX_>&   A,
                                                                     const math::Matrix<value_type, DimX_, DimQ_>&  G,
                                                                     const math::DiagonalMatrix<value_type, DimQ_>& Q)
{
  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    P.thornton(A, G, Q);
  }
  else
  {
    P = math::CovarianceMatrixFull<value_type, DimX_>{
        typename math::CovarianceMatrixFull<value_type, DimX_>::SquareMatrix{A * P * A.transpose() + G * Q * G.transpose()}};
  }
}

template <typename CovarianceMatrixPolicy_>
template <typename UpdateMode_, sint32 DimX_, sint32 DimZ_>
inline void KalmanFilter<CovarianceMatrixPolicy_>::updateState(math::Vector<value_type, DimX_>&              x,
                                                               CovarianceMatrixType<DimX_>&                  P,
                                                               const math::Vector<value_type, DimZ_>&        innovation,
                                                               const math::Matrix<value_type, DimZ_, DimX_>& H,
                                                               const CovarianceMatrixType<DimZ_>&            R)
{
  static_assert(std::is_same_v<UpdateMode_, update_mode::Block> || std::is_same_v<UpdateMode_, update_mode::Sequential>,
                "UpdateMode_ must be update_mode::Block or update_mode::Sequential");
  static_assert(!(CovarianceMatrixPolicy_::is_factored && std::is_same_v<UpdateMode_, update_mode::Block>),
                "Block update would destroy the UDU factorization; use update_mode::Sequential for the factored policy");

  if constexpr (std::is_same_v<UpdateMode_, update_mode::Block>)
  {
    // block EKF update with Joseph stabilized covariance:
    // S = H*P*H' + R ;  K = P*H'*inv(S) ;  x = x + K*nu ;  P = (I-K*H)*P*(I-K*H)' + K*R*K'
    const math::Matrix<value_type, DimX_, DimZ_> PHt{P * H.transpose()};
    math::SquareMatrix<value_type, DimZ_>        S{(H * PHt) + R()};
    S.symmetrize();
    const auto                                   Sinv = S.inverse();
    const math::Matrix<value_type, DimX_, DimZ_> K{PHt * Sinv};

    x += K * innovation;

    const math::SquareMatrix<value_type, DimX_> IKH{math::SquareMatrix<value_type, DimX_>::Identity() - (K * H)};
    P.apaT(IKH);
    P += (K * R()) * K.transpose();
    P.symmetrize();
  }
  else
  {
    // sequential scalar updates on the decorrelated measurement system
    //   nu' = inv(U)*nu ;  H' = inv(U)*H ;  R' = D   with R = U*D*U'
    // (no-op for an already diagonal R), then per row i:
    //   v = P*h_i' ;  s = h_i*v + d_ii ;  x += v*nu_i/s ;  P -= v*v'/s
    // the innovation of later rows is re-linearized around the entry state x0:
    //   nu_i_corr = nu_i - h_i*(x - x0)
    math::Vector<value_type, DimZ_>         nuEff{innovation};
    math::Matrix<value_type, DimZ_, DimX_>  Heff{H};
    math::DiagonalMatrix<value_type, DimZ_> rDiag{};
    if (!detail::decorrelateMeasurement<CovarianceMatrixPolicy_>(nuEff, Heff, rDiag, R))
    {
      return; // defensive: R could not be decomposed, skip the update
    }

    const math::Vector<value_type, DimX_> x0{x};
    math::Vector<value_type, DimX_>       hi{};
    for (sint32 i = 0; i < DimZ_; ++i)
    {
      for (sint32 col = 0; col < DimX_; ++col)
      {
        hi.at_unsafe(col) = Heff.at_unsafe(i, col);
      }

      const auto                            Pfull{P()};
      const math::Vector<value_type, DimX_> v{Pfull * hi};
      const value_type                      s = (hi * v) + rDiag.at_unsafe(i);
      if (!(s > static_cast<value_type>(0.0)))
      {
        continue; // defensive: skip measurement rows with a non-positive innovation variance
      }

      const math::Vector<value_type, DimX_> dx{x - x0};
      const value_type                      nuCorr = nuEff.at_unsafe(i) - (hi * dx);
      x += (nuCorr / s) * v;

      // rank-1 downdate P -= v*v'/s via the shared covariance contract: Agee-Turner keeps the
      // UDU factorization intact for the factored policy, the full policy subtracts the
      // symmetric outer product directly
      P.rank1Update(static_cast<value_type>(-1.0) / s, v);
    }
  }
}

} // namespace filter
} // namespace tracking

#endif // D7AF2A2A_FB74_4C03_A85F_5AC092A11582
