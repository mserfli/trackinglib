#ifndef D0D9A45D_92DE_4848_87C8_9B309333C102
#define D0D9A45D_92DE_4848_87C8_9B309333C102

#include "filter/information_filter.h"

#include "math/linalg/covariance_matrix_factored.hpp"
#include "math/linalg/covariance_matrix_full.hpp"
#include "math/linalg/diagonal_matrix.hpp"
#include "math/linalg/matrix.h"
#include "math/linalg/square_matrix.hpp"

#include "math/linalg/matrix_column_view.hpp"
#include "math/linalg/vector.hpp"


namespace tracking
{
namespace filter
{

template <typename FloatType>
template <sint32 DimX, sint32 DimQ>
static void InformationFilter<FloatType>::predictCovariance(math::CovarianceMatrixFull<FloatType, DimX>& Y,
                                                            const math::SquareMatrix<FloatType, DimX>&   A,
                                                            const math::Matrix<FloatType, DimX, DimQ>&   G,
                                                            const math::DiagonalMatrix<FloatType, DimQ>& Q)
{
  assert(Y.isInverse());
  // Discrete kalman filter tutorial
  // GA Terejanu
  // University at Buffalo, Department of Computer Science and Engineering, NY 14260
  // https://scholar.google.com/citations?view_op=view_citation&hl=en&user=Z7LP12kAAAAJ&citation_for_view=Z7LP12kAAAAJ:_FxGoFyzp5QC

  auto invA = A.inverse();
  auto M{Y};
  M.apaT(invA);
  // solve now H * Y = M with H = (I + M * G*Q*G') using QR as H is not symmetric
  const auto H =
      math::SquareMatrix<FloatType, DimX>(math::SquareMatrix<FloatType, DimX>::Identity() + M * (G * Q * G.transpose()));
  H.qrSolve(Y, M);
}

template <typename FloatType>
template <sint32 DimX, sint32 DimQ>
static void InformationFilter<FloatType>::predictCovariance(math::CovarianceMatrixFactored<FloatType, DimX>& Y,
                                                            const math::SquareMatrix<FloatType, DimX>&       A,
                                                            const math::Matrix<FloatType, DimX, DimQ>&       G,
                                                            const math::DiagonalMatrix<FloatType, DimQ>&     Q)
{
  assert(Y.isInverse());
  // Information Formulation of the UDU Kalman Filter
  // Christopher D’Souza and Renato Zanetti
  // https://sites.utexas.edu/renato/files/2018/05/UDU_Information.pdf

  const auto invA     = A.inverse();
  const auto invAMulG = invA * G;

  // apply DimQ times the AgeeTurner Rank-1 update P = P - c*x*x'
  // with Gi=inv(A)*G(:,i) and ci=inv(Gi'*Y*Gi+inv(Q(i,i))) is (1x1) and x=Y*Gi is (nx1)
  FloatType                     ci;
  math::Vector<FloatType, DimX> xi;
  math::Vector<FloatType, DimX> Gi;
  using ColView = math::MatrixColumnView<FloatType, DimX, DimQ>;
  for (sint32 i = 0; i < DimQ; ++i)
  {
    const ColView Gi{invAMulG, i};
    const auto    fullY{Y()};
    xi = fullY * Gi;
    ci = -1 / (1 / Q[i] + Gi * xi);
    Y.rank1Update(ci, xi);
  }
  // propagate factorization by inverse(A)
  Y.apaT(invA.transpose());
}

} // namespace filter
} // namespace tracking

#endif // D0D9A45D_92DE_4848_87C8_9B309333C102
