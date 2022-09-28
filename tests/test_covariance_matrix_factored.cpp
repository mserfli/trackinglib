#ifndef DAA10430_EB68_4A70_84D3_DA8750F1DF05
#define DAA10430_EB68_4A70_84D3_DA8750F1DF05

#include "gtest/gtest.h"
#include "math/linalg/covariance_matrix_full.h"
#include <trackingLib/math/linalg/covariance_matrix_factored.h>

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFactored<float32, 3>;

TEST(CovarianceMatrixFactored, inverse)
{
    // clang-format off
    tracking::math::CovarianceMatrixFactored<float32, 3> cov(
      {{1, -3, 0.0001}, 
       {0,  1, 1.0   }, 
       {0,  0, 1.0   }}, {19, 42, 0.01});

  const tracking::math::CovarianceMatrixFull<float32, 3> expInvMat(
      {{5.263157894736860e-02, 1.578947368421058e-01, -1.579000000000006e-01},
       {1.578947368421058e-01, 4.974937343358415e-01, -4.975095238095257e-01},
       {-1.579000000000006e-01, -4.975095238095257e-01, 1.004975253138095e+02}});
  // clang-format on
  
  // call UUT
  // auto inv = cov.inverse();
  // auto invFull = inv.compose();

  // EXPECT_EQ(expInvMat._data, invFull._data);
}

// NOLINTEND(modernize-use-trailing-return-type)

#endif // DAA10430_EB68_4A70_84D3_DA8750F1DF05
