#include "gtest/gtest.h"
#include "trackingLib/math/linalg/covariance_matrix_full.hpp"


// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFull<float32, 3>;

TEST(CovarianceMatrixFull, compose) // NOLINT
{
  // clang-format off
  const tracking::math::CovarianceMatrixFull<float32, 3> cov(
    {{45, 52, 12},
     {52, 66, 16},
     {12, 16,  4}});
  // clang-format on

  // call UUT
  auto composed = cov();

  for (sint32 i = 0; i < 3; ++i)
  {
    for (sint32 j = 0; j < 3; ++j)
    {
      EXPECT_EQ(cov(i, j), composed(i, j));
    }
  }
}

TEST(CovarianceMatrixFull, apaT) // NOLINT
{
  // clang-format off
  tracking::math::CovarianceMatrixFull<float64, 4> cov(
    {{ 7.095005365385164,  -2.002405585706988,   3.186228227810577,  -1.796274062161563},
     {-2.002405585706988,   1.993403311363277,  -1.262526372481200,   0.223417952190492},
     { 3.186228227810577,  -1.262526372481200,   1.638799986245814,  -0.810174937419070},
     {-1.796274062161563,   0.223417952190492,  -0.810174937419070,   0.821337656508534}});

  const tracking::math::SquareMatrix<float64, 4> A(
    {{6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
     {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
     {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
     {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
  
  const tracking::math::CovarianceMatrixFull<float64, 4> expCov(
    {{ 2.752859732130674,  -1.285839545719224,   0.473376178291911,   4.416126653289711},
     {-1.285839545719224,   1.851835913837325,   0.811096749802654,  -2.464137199670983},
     { 0.473376178291911,   0.811096749802654,   0.967827910529903,   0.408774316031078},
     { 4.416126653289711,  -2.464137199670983,   0.408774316031078,   7.233518281911786}});
  // clang-format on

  // call UUT
  cov.apaT(A);

  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = 0; j < 4; ++j)
    {
      EXPECT_FLOAT_EQ(expCov(i, j), cov(i, j));
    }
  }
}

TEST(CovarianceMatrixFull, apaT_const) // NOLINT
{
  // clang-format off
  const tracking::math::CovarianceMatrixFull<float64, 4> cov(
    {{ 7.095005365385164,  -2.002405585706988,   3.186228227810577,  -1.796274062161563},
     {-2.002405585706988,   1.993403311363277,  -1.262526372481200,   0.223417952190492},
     { 3.186228227810577,  -1.262526372481200,   1.638799986245814,  -0.810174937419070},
     {-1.796274062161563,   0.223417952190492,  -0.810174937419070,   0.821337656508534}});

  const tracking::math::SquareMatrix<float64, 4> A(
    {{6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
     {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
     {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
     {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
  
  const tracking::math::CovarianceMatrixFull<float64, 4> expCov(
    {{ 2.752859732130674,  -1.285839545719224,   0.473376178291911,   4.416126653289711},
     {-1.285839545719224,   1.851835913837325,   0.811096749802654,  -2.464137199670983},
     { 0.473376178291911,   0.811096749802654,   0.967827910529903,   0.408774316031078},
     { 4.416126653289711,  -2.464137199670983,   0.408774316031078,   7.233518281911786}});
  // clang-format on

  // call UUT
  auto res = cov.apaT(A);

  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = 0; j < 4; ++j)
    {
      EXPECT_FLOAT_EQ(expCov(i, j), res(i, j));
    }
  }
}

TEST(CovarianceMatrixFull, setVariance) // NOLINT
{
  // clang-format off
  tracking::math::CovarianceMatrixFull<float32, 3> cov(
    {{45,  76,  72},
     {76, 132, 120},
     {72, 120, 144}});

  const tracking::math::CovarianceMatrixFull<float32, 3> expMat(
    {{4,   0,   0},
     {0, 132, 120},
     {0, 120, 144}});

  // call UUT
  cov.setVariance(0,4);

  // verify
  const auto& fullCov = cov;
  for(sint32 i=0; i<3; ++i)
  {
    for(sint32 j=0; j<3; ++j)
    {
      EXPECT_FLOAT_EQ(expMat(i,j), fullCov(i, j));
    }
  }
}


