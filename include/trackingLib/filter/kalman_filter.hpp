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

template <typename CovarianceMatrixPolicy_>
template <sint32 DimX_, sint32 DimQ_>
inline void KalmanFilter<CovarianceMatrixPolicy_>::predictCovariance(CovarianceMatrixType<DimX_>&                  P,
                                                                     const math::SquareMatrix<FloatType, DimX_>&   A,
                                                                     const math::Matrix<FloatType, DimX_, DimQ_>&  G,
                                                                     const math::DiagonalMatrix<FloatType, DimQ_>& Q)
{
  if constexpr (CovarianceMatrixPolicy_::is_factored)
  {
    P.thornton(A, G, Q);
  }
  else
  {
    P = math::CovarianceMatrixFull<FloatType, DimX_>{
        typename math::CovarianceMatrixFull<FloatType, DimX_>::SquareMatrix{A * P * A.transpose() + G * Q * G.transpose()}};
  }
}

} // namespace filter
} // namespace tracking

#endif // D7AF2A2A_FB74_4C03_A85F_5AC092A11582
