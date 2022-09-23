#ifndef kalman_filter_h
#define kalman_filter_h

#include "base/covariance_matrix_factored.h"
#include "base/covariance_matrix_full.h"

namespace tracking
{
namespace filter
{

template <typename FloatType>
struct KalmanFilter
{
  template <sint32 DimX, sint32 DimQ>
  static void predictCovariance(base::CovarianceMatrixFull<FloatType, DimX>& P,
                                const base::SquareMatrix<FloatType, DimX>&   A,
                                const base::Matrix<FloatType, DimX, DimQ>&   G,
                                const base::DiagonalMatrix<FloatType, DimQ>& Q)
  {
    P = A * P * A.transpose() + G * Q * G.transpose();
  }

  // prediction for UD factored covariance
  template <sint32 DimX, sint32 DimQ>
  static void predictCovariance(base::CovarianceMatrixFactored<FloatType, DimX>& P,
                                const base::SquareMatrix<FloatType, DimX>&       A,
                                const base::Matrix<FloatType, DimX, DimQ>&       G,
                                const base::DiagonalMatrix<FloatType, DimQ>&     Q)
  {
    // modifiedGramSchmidt
  }
};

} // namespace filter
} // namespace tracking

#endif /* kalman_filter_h */
