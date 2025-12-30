#ifndef D0D9A45D_92DE_4848_87C8_9B309333C102
#define D0D9A45D_92DE_4848_87C8_9B309333C102

#include "filter/information_filter.h"

#include "math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/matrix_column_view.hpp"         // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"              // IWYU pragma: keep
#include "math/linalg/vector.hpp"                     // IWYU pragma: keep


namespace tracking
{
namespace filter
{

template <typename FloatType_>
template <sint32 DimX_, sint32 DimQ_>
void InformationFilter<FloatType_>::predictCovariance(math::CovarianceMatrixFull<FloatType_, DimX_>& Y,
                                                      const math::SquareMatrix<FloatType_, DimX_>&   A,
                                                      const math::Matrix<FloatType_, DimX_, DimQ_>&  G,
                                                      const math::DiagonalMatrix<FloatType_, DimQ_>& Q)
{
  // Discrete kalman filter tutorial
  // GA Terejanu
  // University at Buffalo, Department of Computer Science and Engineering, NY 14260
  // https://scholar.google.com/citations?view_op=view_citation&hl=en&user=Z7LP12kAAAAJ&citation_for_view=Z7LP12kAAAAJ:_FxGoFyzp5QC
  auto M{Y};
  // calc M = invA'*Y*invA as Y is the inverse of P
  const math::SquareMatrix<FloatType_, DimX_, false> invA = A.inverse();
  M.apaT(math::SquareMatrix<FloatType_, DimX_, true>{invA.transpose()});
  // solve now H * Y = M with H = (I + M * G*Q*G') using QR as H is not symmetric
  const auto H =
      math::SquareMatrix<FloatType_, DimX_>(math::SquareMatrix<FloatType_, DimX_>::Identity() + M * (G * Q * G.transpose()));
  auto cov = H.qrSolve(M);

  // symmetrize
  cov += cov.transpose();
  cov *= static_cast<FloatType_>(0.5);
  Y = math::CovarianceMatrixFull<FloatType_, DimX_>{std::move(cov)};
}

template <typename FloatType_>
template <sint32 DimX_, sint32 DimQ_>
void InformationFilter<FloatType_>::predictCovariance(math::CovarianceMatrixFactored<FloatType_, DimX_>& Y,
                                                      const math::SquareMatrix<FloatType_, DimX_>&       A,
                                                      const math::Matrix<FloatType_, DimX_, DimQ_>&      G,
                                                      const math::DiagonalMatrix<FloatType_, DimQ_>&     Q)
{
  // Information Formulation of the UDU Kalman Filter
  // Christopher D’Souza and Renato Zanetti
  // https://sites.utexas.edu/renato/files/2018/05/UDU_Information.pdf
  const math::SquareMatrix<FloatType_, DimX_, false>  invA     = A.inverse();
  const math::Matrix<FloatType_, DimX_, DimQ_, false> invAMulG = invA * G;

  // apply DimQ times the Rank-1 update P = P - c*x*x'
  // with Gi=inv(A)*G(:,i) and ci=inv(Gi'*Y*Gi+inv(Q(i,i))) is (1x1) and x=Y*Gi is (nx1)
  FloatType_                      ci;
  math::Vector<FloatType_, DimX_> xi;
  using ColView = math::MatrixColumnView<FloatType_, DimX_, DimQ_, false>;
  for (sint32 i = 0; i < DimQ_; ++i)
  {
    const ColView Gi{invAMulG, i};
    const auto    fullY{Y()};
    xi = fullY * Gi;
    ci = -1 / (1 / Q.at_unsafe(i) + Gi * xi);

    Y.rank1Update(ci, xi);
  }
  // propagate factorization by inv(A)'
  const math::SquareMatrix<FloatType_, DimX_, true> invAT{invA.transpose()};
  Y.apaT(invAT);
}

} // namespace filter
} // namespace tracking

#endif // D0D9A45D_92DE_4848_87C8_9B309333C102
