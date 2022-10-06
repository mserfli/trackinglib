#ifndef CD00E333_97EF_4391_B880_C543B45E2D3F
#define CD00E333_97EF_4391_B880_C543B45E2D3F

#include "base/first_include.h"
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/square_matrix.hpp"

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
    auto invA = A.inverse();
    auto M    = invA.transpose() * Y * invA;
    auto H    = math::SquareMatrix<FloatType, DimX>::Identity() + M * (G * Q * G.transpose());
    auto invH = H.inverse();
    Y         = invH * M;
  }

  // prediction for UD factored covariance
  template <sint32 DimX, sint32 DimQ>
  static void predictCovariance(math::CovarianceMatrixFactored<FloatType, DimX>& P,
                                const math::SquareMatrix<FloatType, DimX>&       A,
                                const math::Matrix<FloatType, DimX, DimQ>&       G,
                                const math::DiagonalMatrix<FloatType, DimQ>&     Q)
  {
    // modifiedGramSchmidt
  }
};

} // namespace filter
} // namespace tracking
#endif // CD00E333_97EF_4391_B880_C543B45E2D3F
