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

} // namespace filter
} // namespace tracking

#endif // D0D9A45D_92DE_4848_87C8_9B309333C102
