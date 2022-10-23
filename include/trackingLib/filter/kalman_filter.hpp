#ifndef D7AF2A2A_FB74_4C03_A85F_5AC092A11582
#define D7AF2A2A_FB74_4C03_A85F_5AC092A11582

#include "filter/kalman_filter.h"

#include "math/linalg/covariance_matrix_factored.hpp"
#include "math/linalg/covariance_matrix_full.hpp"
#include "math/linalg/diagonal_matrix.hpp"
#include "math/linalg/matrix.h"
#include "math/linalg/square_matrix.hpp"

namespace tracking
{
namespace filter
{

template <typename FloatType>
template <sint32 DimX, sint32 DimQ>
inline static void KalmanFilter<FloatType>::predictCovariance(math::CovarianceMatrixFull<FloatType, DimX>& P,
                                                              const math::SquareMatrix<FloatType, DimX>&   A,
                                                              const math::Matrix<FloatType, DimX, DimQ>&   G,
                                                              const math::DiagonalMatrix<FloatType, DimQ>& Q)
{
  P = math::CovarianceMatrixFull<FloatType, DimX>(A * P * A.transpose() + G * Q * G.transpose());
}

template <typename FloatType>
template <sint32 DimX, sint32 DimQ>
inline static void KalmanFilter<FloatType>::predictCovariance(math::CovarianceMatrixFactored<FloatType, DimX>& P,
                                                              const math::SquareMatrix<FloatType, DimX>&       A,
                                                              const math::Matrix<FloatType, DimX, DimQ>&       G,
                                                              const math::DiagonalMatrix<FloatType, DimQ>&     Q)
{
  P.thornton(A, G, Q);
}

} // namespace filter
} // namespace tracking

#endif // D7AF2A2A_FB74_4C03_A85F_5AC092A11582
