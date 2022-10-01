#ifndef DAA10430_EB68_4A70_84D3_DA8750F1DF05
#define DAA10430_EB68_4A70_84D3_DA8750F1DF05

#include "gtest/gtest.h"
#include "math/linalg/covariance_matrix_full.h"
#include <trackingLib/math/linalg/covariance_matrix_factored.h>

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFactored<float32, 3>;

TEST(CovarianceMatrixFactored, compose)
{
  // clang-format off
  tracking::math::CovarianceMatrixFactored<float32, 3> cov(
    {{1,2,3}, 
     {0,4,5}, 
     {0,0,6}}, {1, 2, 4});

  const tracking::math::CovarianceMatrixFull<float32, 3> expMat(
    {{45,  76,  72},
     {76, 132, 120},
     {72, 120, 144}});
  // clang-format on

  // call UUT
  auto composed = cov();

  for (sint32 i = 0; i < 3; ++i)
  {
    for (sint32 j = 0; j < 3; ++j)
    {
      EXPECT_EQ(expMat(i, j), composed(i, j));
    }
  }
}

TEST(CovarianceMatrixFactored, composeInverse)
{
  // clang-format off
  tracking::math::CovarianceMatrixFactored<float32, 3> cov(
    {{1,2,3}, 
     {0,4,5}, 
     {0,0,6}}, {1, 2, 4}, true);

  const tracking::math::CovarianceMatrixFull<float32, 3> expMat(
    {{1,  2,   3},
     {2, 36,  46},
     {3, 46, 203}});
  // clang-format on

  // call UUT
  auto composed = cov();

  for (sint32 i = 0; i < 3; ++i)
  {
    for (sint32 j = 0; j < 3; ++j)
    {
      EXPECT_EQ(expMat(i, j), composed(i, j));
    }
  }
}

TEST(CovarianceMatrixFactored, inverse)
{
  // clang-format off
  tracking::math::CovarianceMatrixFactored<float32, 3> cov(
    {{1, 2, 3}, 
     {0, 4, 5}, 
     {0, 0, 6}}, {1, 2, 4});

  const tracking::math::CovarianceMatrixFactored<float32, 3> expInvMat(
    {{1, -0.5,  -0.083333333333333},
     {0,  0.25, -0.208333333333333},
     {0,  0,     0.166666666666667}}, {1, 0.5, 0.25});
  // clang-format on

  // call UUT
  auto inv = cov.inverse();

  EXPECT_TRUE(inv._isInverse);
  for (sint32 i = 0; i < 3; ++i)
  {
    EXPECT_FLOAT_EQ(expInvMat._d[i], inv._d[i]);
    for (sint32 j = i; j < 3; ++j)
    {
      EXPECT_FLOAT_EQ(expInvMat._u(i, j), inv._u(i, j));
    }
  }
}

TEST(CovarianceMatrixFactored, calcCovarianceElement)
{
  // clang-format off
  tracking::math::CovarianceMatrixFactored<float32, 3> cov(
    {{1, 2, 3}, 
     {0, 4, 5}, 
     {0, 0, 6}}, {1, 2, 4});

  const float32 expCovElem = 120.0F;
  // clang-format on

  // call UUT
  auto res = cov(1,2);

  EXPECT_EQ(res, cov(2,1));
  EXPECT_EQ(res, expCovElem);
}

// NOLINTEND(modernize-use-trailing-return-type)

#endif // DAA10430_EB68_4A70_84D3_DA8750F1DF05
