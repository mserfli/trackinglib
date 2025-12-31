#ifndef D7AF2A2A_FB74_4C03_A85F_5AC092A11582
#define D7AF2A2A_FB74_4C03_A85F_5AC092A11582

#include "filter/kalman_filter.h"

#include "math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include "math/linalg/diagonal_matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/matrix.hpp"                     // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"              // IWYU pragma: keep

namespace tracking
{
namespace filter
{

template <typename FloatType_>
template <sint32 DimX_, sint32 DimQ_>
inline void KalmanFilter<FloatType_>::predictCovariance(math::CovarianceMatrixFull<FloatType_, DimX_>& P,
                                                        const math::SquareMatrix<FloatType_, DimX_>&   A,
                                                        const math::Matrix<FloatType_, DimX_, DimQ_>&  G,
                                                        const math::DiagonalMatrix<FloatType_, DimQ_>& Q)
{
  P = math::CovarianceMatrixFull<FloatType_, DimX_>{
      typename math::CovarianceMatrixFull<FloatType_, DimX_>::SquareMatrix{A * P * A.transpose() + G * Q * G.transpose()}};
}

template <typename FloatType_>
template <sint32 DimX_, sint32 DimQ_>
inline void KalmanFilter<FloatType_>::predictCovariance(math::CovarianceMatrixFactored<FloatType_, DimX_>& P,
                                                        const math::SquareMatrix<FloatType_, DimX_>&       A,
                                                        const math::Matrix<FloatType_, DimX_, DimQ_>&      G,
                                                        const math::DiagonalMatrix<FloatType_, DimQ_>&     Q)
{
  P.thornton(A, G, Q);
}

} // namespace filter
} // namespace tracking

#endif // D7AF2A2A_FB74_4C03_A85F_5AC092A11582
