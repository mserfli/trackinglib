#ifndef CD00E333_97EF_4391_B880_C543B45E2D3F
#define CD00E333_97EF_4391_B880_C543B45E2D3F

#include "base/first_include.h"
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/square_matrix.hpp"
#include "math/linalg/vector.h"

namespace tracking
{
namespace filter
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <typename FloatType>
struct InformationFilter
{
  template <sint32 DimX, sint32 DimQ>
  static void predictCovariance(math::CovarianceMatrixFull<FloatType, DimX>& Y,
                                const math::SquareMatrix<FloatType, DimX>&   A,
                                const math::Matrix<FloatType, DimX, DimQ>&   G,
                                const math::DiagonalMatrix<FloatType, DimQ>& Q)
  {
    assert(Y.isInverse());
    // Discrete kalman filter tutorial
    // GA Terejanu
    // University at Buffalo, Department of Computer Science and Engineering, NY 14260
    // https://scholar.google.com/citations?view_op=view_citation&hl=en&user=Z7LP12kAAAAJ&citation_for_view=Z7LP12kAAAAJ:_FxGoFyzp5QC

    Y.print();
    A.print();
    G.print();
    Q.print();

    auto invA = A.inverse();
    auto M{Y};
    M.apaT(invA);
    // solve now H * Y = M with H = (I + M * G*Q*G') using QR as H is not symmetric
    const auto H =
        math::SquareMatrix<FloatType, DimX>(math::SquareMatrix<FloatType, DimX>::Identity() + M * (G * Q * G.transpose()));
    H.qrSolve(Y, M);
    Y.print();
  }

  // prediction for UD factored covariance
  template <sint32 DimX, sint32 DimQ>
  static void predictCovariance(math::CovarianceMatrixFactored<FloatType, DimX>& Y,
                                const math::SquareMatrix<FloatType, DimX>&       A,
                                const math::Matrix<FloatType, DimX, DimQ>&       G,
                                const math::DiagonalMatrix<FloatType, DimQ>&     Q)
  {
    assert(Y.isInverse());
    // Information Formulation of the UDU Kalman Filter
    // Christopher D’Souza and Renato Zanetti
    // https://sites.utexas.edu/renato/files/2018/05/UDU_Information.pdf

    Y.print();
    A.print();
    G.print();
    Q.print();
#if 0
    auto P = Y.inverse();
    P.thornton(A, G, Q);
    Y = P.inverse();
    Y.print();  // close to CovarianceMatrixFull solution
#else
    {
      const auto cov = Y();
      const auto inv = math::CovarianceMatrixFull<FloatType, DimQ>((G.transpose() * cov * G) + Q.inverse()).inverse();
      const auto tmp = cov * G;
      auto res = math::CovarianceMatrixFull<FloatType, DimX>(cov - (tmp * inv) * tmp.transpose(), true);
      res.print();
      auto tmp2 = res;
      res.apaT(A.inverse());
      res.print(); // even closer to CovarianceMatrixFull solution, but without keeping the UDU structure
      tmp2 -= res;
      tmp2.print();

    }

    // apply DimQ times the AgeeTurner Rank-1 update P = P - c*x*x'
    // with ci=Gi'*Y*Gi+Q(i,i) is (1x1) and x=Y*Gi is (nx1)
    math::Vector<FloatType, DimX> x;
    math::Vector<FloatType, DimX> Gi;
    for (sint32 i = 0; i < DimQ; ++i)
    {
      FloatType ci{Q[i]};
      for (sint32 j = 0; j < DimX; ++j)
      {
        Gi[j] = G(j, i);
      }
      x                 = Y() * Gi;
      const auto scalar = Gi.transpose() * x;
      ci += scalar(0, 0);
      Y.rank1Update(ci, x);
    }
    // propagate factorization by inverse(A)
    Y.apaT(A.inverse());
    Y.print();
#endif
  }
};

} // namespace filter
} // namespace tracking

#endif // CD00E333_97EF_4391_B880_C543B45E2D3F
