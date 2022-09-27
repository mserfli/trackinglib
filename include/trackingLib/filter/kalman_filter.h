#ifndef DA0115C7_88CB_4774_A6A3_54764AF1BF9D
#define DA0115C7_88CB_4774_A6A3_54764AF1BF9D

#include "base/first_include.h"
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"

namespace tracking
{
namespace filter
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <typename FloatType>
struct KalmanFilter
{
  template <sint32 DimX, sint32 DimQ>
  static void predictCovariance(math::CovarianceMatrixFull<FloatType, DimX>& P,
                                const math::SquareMatrix<FloatType, DimX>&   A,
                                const math::Matrix<FloatType, DimX, DimQ>&   G,
                                const math::DiagonalMatrix<FloatType, DimQ>& Q)
  {
    P = A * P * A.transpose() + G * Q * G.transpose();
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

#endif // DA0115C7_88CB_4774_A6A3_54764AF1BF9D
