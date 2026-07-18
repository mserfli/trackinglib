#ifndef D0D9A45D_92DE_4848_87C8_9B309333C102
#define D0D9A45D_92DE_4848_87C8_9B309333C102

#include "filter/information_filter.h"

#include "filter/measurement_decorrelation.hpp"
#include "math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/matrix_column_view.hpp"         // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"              // IWYU pragma: keep
#include "math/linalg/vector.hpp"                     // IWYU pragma: keep
#include <type_traits>                                // IWYU pragma: keep


namespace tracking
{
namespace filter
{

template <typename CovarianceMatrixPolicy_>
template <sint32 DimX_, sint32 DimQ_>
void InformationFilter<CovarianceMatrixPolicy_>::predictCovariance(CovarianceMatrixType<DimX_>&                   Y,
                                                                   const math::SquareMatrix<value_type, DimX_>&   A,
                                                                   const math::Matrix<value_type, DimX_, DimQ_>&  G,
                                                                   const math::DiagonalMatrix<value_type, DimQ_>& Q)
{
  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    // Information Formulation of the UDU Kalman Filter
    // Christopher D’Souza and Renato Zanetti
    // https://sites.utexas.edu/renato/files/2018/05/UDU_Information.pdf
    const math::SquareMatrix<value_type, DimX_, false>  invA     = A.inverse();
    const math::Matrix<value_type, DimX_, DimQ_, false> invAMulG = invA * G;

    // apply DimQ times the Rank-1 update P = P - c*x*x'
    // with Gi=inv(A)*G(:,i) and ci=inv(Gi'*Y*Gi+inv(Q(i,i))) is (1x1) and x=Y*Gi is (nx1)
    value_type                      ci;
    math::Vector<value_type, DimX_> xi;
    using ColView = math::MatrixColumnView<value_type, DimX_, DimQ_, false>;
    for (sint32 i = 0; i < DimQ_; ++i)
    {
      const ColView Gi{invAMulG, i};
      const auto    fullY{Y()};
      xi = fullY * Gi;
      ci = -1 / (1 / Q.at_unsafe(i) + Gi * xi);

      Y.rank1Update(ci, xi);
    }
    // propagate factorization by inv(A)'
    const math::SquareMatrix<value_type, DimX_, true> invAT{invA.transpose()};
    Y.apaT(invAT);
  }
  else
  {
    // Discrete kalman filter tutorial
    // GA Terejanu
    // University at Buffalo, Department of Computer Science and Engineering, NY 14260
    // https://scholar.google.com/citations?view_op=view_citation&hl=en&user=Z7LP12kAAAAJ&citation_for_view=Z7LP12kAAAAJ:_FxGoFyzp5QC
    auto M{Y};
    // calc M = invA'*Y*invA as Y is the inverse of P
    const math::SquareMatrix<value_type, DimX_, false> invA = A.inverse();
    M.apaT(math::SquareMatrix<value_type, DimX_, true>{invA.transpose()});
    // solve now H * Y = M with H = (I + M * G*Q*G') using QR as H is not symmetric
    const auto H =
        math::SquareMatrix<value_type, DimX_>(math::SquareMatrix<value_type, DimX_>::Identity() + M * (G * Q * G.transpose()));
    math::SquareMatrix cov = H.qrSolve(std::move(M));
    cov.symmetrize();

    // prevent destroying the Information matrix, e.g. removing information from a zero Y matrix (no information)
    if (cov.isPositiveSemiDefinite())
    {
      Y = math::CovarianceMatrixFull<value_type, DimX_>{std::move(cov)};
    }
  }
}

template <typename CovarianceMatrixPolicy_>
template <typename UpdateMode_, sint32 DimX_, sint32 DimZ_>
inline void InformationFilter<CovarianceMatrixPolicy_>::updateState(math::Vector<value_type, DimX_>&              y,
                                                                    CovarianceMatrixType<DimX_>&                  Y,
                                                                    const math::Vector<value_type, DimZ_>&        z,
                                                                    const math::Matrix<value_type, DimZ_, DimX_>& H,
                                                                    const CovarianceMatrixType<DimZ_>&            R)
{
  static_assert(std::is_same_v<UpdateMode_, update_mode::Block> || std::is_same_v<UpdateMode_, update_mode::Sequential>,
                "UpdateMode_ must be update_mode::Block or update_mode::Sequential");
  static_assert(!(CovarianceMatrixPolicy_::is_factored && std::is_same_v<UpdateMode_, update_mode::Block>),
                "Block update would destroy the UDU factorization; use update_mode::Sequential for the factored policy");

  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    // decorrelate the measurement system first (no-op for an already diagonal R):
    //   z' = inv(U)*z ;  H' = inv(U)*H ;  R' = D   with R = U*D*U'
    // then apply DimZ times the additive rank-1 update Y = Y + c*h*h' with c = 1/d_ii
    // (measurement adds information)
    math::Vector<value_type, DimZ_>         zEff{z};
    math::Matrix<value_type, DimZ_, DimX_>  Heff{H};
    math::DiagonalMatrix<value_type, DimZ_> rDiag{};
    if (!detail::decorrelateMeasurement<CovarianceMatrixPolicy_>(zEff, Heff, rDiag, R))
    {
      return; // defensive: R could not be decomposed, skip the update
    }

    math::Vector<value_type, DimX_> hi{};
    for (sint32 i = 0; i < DimZ_; ++i)
    {
      const value_type dii = rDiag.at_unsafe(i);
      if (!(dii > static_cast<value_type>(0.0)))
      {
        continue; // defensive: skip measurement rows with a non-positive variance
      }

      for (sint32 col = 0; col < DimX_; ++col)
      {
        hi.at_unsafe(col) = Heff.at_unsafe(i, col);
      }

      const value_type ci = static_cast<value_type>(1.0) / dii;
      Y.rank1Update(ci, hi);
      y += (ci * zEff.at_unsafe(i)) * hi;
    }
  }
  else
  {
    // matrix-form additive update: y += H'*inv(R)*z ;  Y += H'*inv(R)*H
    const auto invR = R.inverse();
    if (!invR.has_value())
    {
      return; // skip the update if R is not invertible
    }

    const auto HtInvR = H.transpose() * invR.value();
    y += HtInvR * z;
    Y += HtInvR * H;
    Y.symmetrize();
  }
}

} // namespace filter
} // namespace tracking

#endif // D0D9A45D_92DE_4848_87C8_9B309333C102
