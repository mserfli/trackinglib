#include "gtest/gtest.h"
#include "trackingLib/math/linalg/covariance_matrix_factored.h"
#include "trackingLib/math/linalg/covariance_matrix_full.h"
#include "trackingLib/math/linalg/square_matrix.hpp"

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
  auto res = cov(1, 2);

  EXPECT_EQ(res, cov(2, 1));
  EXPECT_EQ(res, expCovElem);
}

TEST(CovarianceMatrixFactored, apaT)
{
  // clang-format off
  tracking::math::CovarianceMatrixFactored<float64, 4> cov(
    // u=
    {{1.000000000000000,   0.378176125484376,   1.684500252612056,  -2.187010479706283},
     {                0,   1.000000000000000,  -1.241186420682381,   0.272017178830240},
     {                0,                   0,   1.000000000000000,  -0.986409098619786},
     {                0,                   0,                   0,   1.000000000000000}}, 
    // d=
    { 0.692626242247276,   0.639133727238215,   0.839636056501929,   0.821337656508534});

  const tracking::math::SquareMatrix<float64, 4> A(
    {{6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
     {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
     {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
     {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
  
  const tracking::math::CovarianceMatrixFactored<float64, 4> expCov(
    // u=
    {{1.000000000000000,  -0.117162650062257,   0.236910463595170,   0.610508812057989},
     {                0,   1.000000000000000,   1.005949148623783,  -0.340655418792931},
     {                0,                   0,   1.000000000000000,   0.056511133324051},
     {                0,                   0,                   0,   1.000000000000000}}, 
    // d=
    { 0.002976797231001,   0.056412527384831,   0.944727610657223,   7.233518281911786});
  // clang-format on

  // call UUT
  cov.apaT(A);

  for (sint32 i = 0; i < 4; ++i)
  {
    EXPECT_FLOAT_EQ(expCov._d[i], cov._d[i]);
    for (sint32 j = i; j < 4; ++j)
    {
      EXPECT_FLOAT_EQ(expCov._u(i, j), cov._u(i, j));
    }
  }
}

TEST(CovarianceMatrixFactored, apaT_const)
{
  // clang-format off
  const tracking::math::CovarianceMatrixFactored<float64, 4> cov(
    // u=
    {{1.000000000000000,   0.378176125484376,   1.684500252612056,  -2.187010479706283},
     {                0,   1.000000000000000,  -1.241186420682381,   0.272017178830240},
     {                0,                   0,   1.000000000000000,  -0.986409098619786},
     {                0,                   0,                   0,   1.000000000000000}}, 
    // d=
    { 0.692626242247276,   0.639133727238215,   0.839636056501929,   0.821337656508534});

  const tracking::math::SquareMatrix<float64, 4> A(
    {{6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
     {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
     {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
     {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
  
  const tracking::math::CovarianceMatrixFactored<float64, 4> expCov(
    // u=
    {{1.000000000000000,  -0.117162650062257,   0.236910463595170,   0.610508812057989},
     {                0,   1.000000000000000,   1.005949148623783,  -0.340655418792931},
     {                0,                   0,   1.000000000000000,   0.056511133324051},
     {                0,                   0,                   0,   1.000000000000000}}, 
    // d=
    { 0.002976797231001,   0.056412527384831,   0.944727610657223,   7.233518281911786});
  // clang-format on

  // call UUT
  auto res = cov.apaT(A);

  for (sint32 i = 0; i < 4; ++i)
  {
    EXPECT_FLOAT_EQ(expCov._d[i], res._d[i]);
    for (sint32 j = i; j < 4; ++j)
    {
      EXPECT_FLOAT_EQ(expCov._u(i, j), res._u(i, j));
    }
  }
}

TEST(CovarianceMatrixFactored, setVariance)
{
  // clang-format off
  tracking::math::CovarianceMatrixFactored<float32, 3> cov(
    {{1,2,3}, 
     {0,4,5}, 
     {0,0,6}}, {1, 2, 4}, false);

  const tracking::math::CovarianceMatrixFull<float32, 3> expMat(
    {{4,   0,   0},
     {0, 132, 120},
     {0, 120, 144}});

  // call UUT
  cov.setVariance(0,4);

  // verify
  const auto fullCov = cov();
  for(sint32 i=0; i<3; ++i)
  {
    for(sint32 j=0; j<3; ++j)
    {
      EXPECT_FLOAT_EQ(expMat(i,j), fullCov(i, j));
    }
  }
}

// NOLINTEND(modernize-use-trailing-return-type)
