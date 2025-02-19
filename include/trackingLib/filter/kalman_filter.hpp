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

template <typename FloatType_>
template <sint32 DimX_, sint32 DimQ_>
inline void KalmanFilter<FloatType_>::predictCovariance(math::CovarianceMatrixFull<FloatType_, DimX_>& P,
                                                        const math::SquareMatrix<FloatType_, DimX_>&   A,
                                                        const math::Matrix<FloatType_, DimX_, DimQ_>&  G,
                                                        const math::DiagonalMatrix<FloatType_, DimQ_>& Q)
{
  P = math::CovarianceMatrixFull<FloatType_, DimX_>{A * P * A.transpose() + G * Q * G.transpose()};
  P += P.transpose();
  P *= static_cast<FloatType_>(0.5);
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
