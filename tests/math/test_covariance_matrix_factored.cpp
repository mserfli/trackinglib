#include "gtest/gtest.h"
#include "trackingLib/math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep


// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFactored<float32, 3>;

TEST(CovarianceMatrixFactored, compose) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1,2,3}, 
    {0,1,4}, 
    {0,0,1}}, {1, 2, 4}, false);

  const auto expMat = tracking::math::CovarianceMatrixFull<float32, 3>::FromList({
    {45, 52, 12},
    {52, 66, 16},
    {12, 16,  4}}, false);
  // clang-format on

  // call UUT
  const auto composed = cov();

  EXPECT_EQ(expMat.isInverse(), composed.isInverse());
  EXPECT_EQ(expMat._data, composed._data);
}

TEST(CovarianceMatrixFactored, composeInverse) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1,2,3}, 
    {0,1,4}, 
    {0,0,1}}, {1, 2, 4}, true);

  const auto expMat = tracking::math::CovarianceMatrixFull<float32, 3>::FromList({
    {1,  2,  3},
    {2,  6, 14},
    {3, 14, 45}}, true);
  // clang-format on

  // call UUT
  const auto composed = cov();

  EXPECT_EQ(expMat.isInverse(), composed.isInverse());
  EXPECT_EQ(expMat._data, composed._data);
}

TEST(CovarianceMatrixFactored, inverse_forward) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1,2,3}, 
    {0,1,4}, 
    {0,0,1}}, {1, 2, 4}, false);
  const auto expFullCov = tracking::math::CovarianceMatrixFull<float32, 3>::FromList({
    {45, 52, 12},
    {52, 66, 16},
    {12, 16,  4}}, false);

  const auto expInvCov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1, -2,  5},
    {0,  1, -4},
    {0,  0,  1}}, {1, 0.5, 0.25}, true);
  const auto expInvFullCov = tracking::math::CovarianceMatrixFull<float32, 3>::FromList({
    { 1.0,  -2.0,  5.00},
    {-2.0,   4.5,-12.00},
    { 5.0, -12.0, 33.25}}, true);
  // clang-format on
  EXPECT_EQ(expFullCov._data, cov()._data);
  EXPECT_EQ(expInvFullCov._data, expInvCov()._data);

  // call UUT
  const auto inv = cov.inverse().value();

  EXPECT_EQ(expInvCov._isInverse, inv._isInverse);
  EXPECT_EQ(expInvCov._d._data, inv._d._data);
  EXPECT_EQ(expInvCov._u._data, inv._u._data);
}

TEST(CovarianceMatrixFactored, inverse_reverse) // NOLINT
{
  // clang-format off
  auto covInv = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1, -2,  5},
    {0,  1, -4},
    {0,  0,  1}}, {1, 0.5, 0.25}, true);
  const auto expInvFullCov = tracking::math::CovarianceMatrixFull<float32, 3>::FromList({
    { 1.0,  -2.0,  5.00},
    {-2.0,   4.5,-12.00},
    { 5.0, -12.0, 33.25}}, true);
  
  const auto expCov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1,2,3}, 
    {0,1,4}, 
    {0,0,1}}, {1, 2, 4}, false);
  const auto expFullCov = tracking::math::CovarianceMatrixFull<float32, 3>::FromList({
    {45, 52, 12},
    {52, 66, 16},
    {12, 16,  4}}, false);
  // clang-format on
  EXPECT_EQ(expFullCov._data, expCov()._data);
  EXPECT_EQ(expInvFullCov._data, covInv()._data);

  // call UUT
  const auto inv = covInv.inverse().value();

  EXPECT_EQ(expCov._isInverse, inv._isInverse);
  EXPECT_EQ(expCov._d._data, inv._d._data);
  EXPECT_EQ(expCov._u._data, inv._u._data);
}

struct CovarianceMatrixFactoredWithParams: public testing::TestWithParam<std::tuple<bool, int, int>>
{
  void SetUp() final
  {
    _cov     = std::get<0>(GetParam()) ? _covIn.inverse().value() : _covIn;
    _covFull = _cov();
  }

  // clang-format off
  tracking::math::CovarianceMatrixFactored<float32, 3> _covIn = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1, 2, 3}, 
    {0, 1, 4}, 
    {0, 0, 1}}, {1, 2, 4}, false);

  tracking::math::CovarianceMatrixFactored<float32, 3> _cov{};
  tracking::math::CovarianceMatrixFull<float32, 3> _covFull{};
  // clang-format on
};

TEST_P(CovarianceMatrixFactoredWithParams, calcCovarianceElement) // NOLINT
{
  const auto [isInverse, row, col] = GetParam(); // structured binding since C++17

  // call UUT
  auto res = _cov(row, col);

  EXPECT_EQ(res, _covFull(row, col));
}

INSTANTIATE_TEST_CASE_P(CovarianceMatrixFactored,
                        CovarianceMatrixFactoredWithParams,
                        ::testing::Combine(::testing::Values(false, true), ::testing::Range(0, 3), ::testing::Range(0, 3)));

TEST(CovarianceMatrixFactored, apaT) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::CovarianceMatrixFactored<float64, 4>::FromList({
    {1.000000000000000,   0.378176125484376,   1.684500252612056,  -2.187010479706283},
    {                0,   1.000000000000000,  -1.241186420682381,   0.272017178830240},
    {                0,                   0,   1.000000000000000,  -0.986409098619786},
    {                0,                   0,                   0,   1.000000000000000}}, 
    { 0.692626242247276,   0.639133727238215,   0.839636056501929,   0.821337656508534}, false);

  const auto A = tracking::math::SquareMatrix<float64, 4, true>::FromList({
    {6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
    {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
    {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
    {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
  
  const auto expCov = tracking::math::CovarianceMatrixFactored<float64, 4>::FromList({
    {1.000000000000000,  -0.117162650062257,   0.236910463595170,   0.610508812057989},
    {                0,   1.000000000000000,   1.005949148623783,  -0.340655418792931},
    {                0,                   0,   1.000000000000000,   0.056511133324051},
    {                0,                   0,                   0,   1.000000000000000}}, 
    { 0.002976797231001,   0.056412527384831,   0.944727610657223,   7.233518281911786}, false);
  // clang-format on

  // call UUT
  cov.apaT(A);

  EXPECT_EQ(expCov._isInverse, cov._isInverse);
  for (sint32 i = 0; i < 4; ++i)
  {
    EXPECT_FLOAT_EQ(expCov._d.at_unsafe(i), cov._d.at_unsafe(i));
    for (sint32 j = i; j < 4; ++j)
    {
      EXPECT_FLOAT_EQ(expCov._u.at_unsafe(i, j), cov._u.at_unsafe(i, j));
    }
  }
}

TEST(CovarianceMatrixFactored, apaT_const) // NOLINT
{
  // clang-format off
  const auto cov = tracking::math::CovarianceMatrixFactored<float64, 4>::FromList({
    {1.000000000000000,   0.378176125484376,   1.684500252612056,  -2.187010479706283},
    {                0,   1.000000000000000,  -1.241186420682381,   0.272017178830240},
    {                0,                   0,   1.000000000000000,  -0.986409098619786},
    {                0,                   0,                   0,   1.000000000000000}}, 
    { 0.692626242247276,   0.639133727238215,   0.839636056501929,   0.821337656508534}, false);

  const auto A = tracking::math::SquareMatrix<float64, 4, true>::FromList({
    {6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
    {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
    {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
    {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
  
  const auto expCov = tracking::math::CovarianceMatrixFactored<float64, 4>::FromList({
    {1.000000000000000,  -0.117162650062257,   0.236910463595170,   0.610508812057989},
    {                0,   1.000000000000000,   1.005949148623783,  -0.340655418792931},
    {                0,                   0,   1.000000000000000,   0.056511133324051},
    {                0,                   0,                   0,   1.000000000000000}}, 
    { 0.002976797231001,   0.056412527384831,   0.944727610657223,   7.233518281911786}, false);
  // clang-format on

  // call UUT
  auto res = cov.apaT(A);

  EXPECT_EQ(expCov._isInverse, cov._isInverse);
  for (sint32 i = 0; i < 4; ++i)
  {
    EXPECT_FLOAT_EQ(expCov._d.at_unsafe(i), res._d.at_unsafe(i));
    for (sint32 j = i; j < 4; ++j)
    {
      EXPECT_FLOAT_EQ(expCov._u.at_unsafe(i, j), res._u.at_unsafe(i, j));
    }
  }
}

#if 0 // TODO fix this if required, apaT with inverse is used in MMCV and MMCA tests with success
TEST(CovarianceMatrixFactored, apaT_isInverse) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::CovarianceMatrixFactored<float64, 4>::FromList({
    {1.000000000000000,   0.378176125484376,   1.684500252612056,  -2.187010479706283},
    {                0,   1.000000000000000,  -1.241186420682381,   0.272017178830240},
    {                0,                   0,   1.000000000000000,  -0.986409098619786},
    {                0,                   0,                   0,   1.000000000000000}}, 
    { 0.692626242247276,   0.639133727238215,   0.839636056501929,   0.821337656508534}, true);

  const auto A = tracking::math::SquareMatrix<float64, 4, true>::FromList({
    {6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
    {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
    {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
    {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
  
  const auto expCov = tracking::math::CovarianceMatrixFactored<float64, 4>::FromList({
    { 1.000000000000000, -0.589195457221914, -0.781090204042029, -2.667605868133106},
    {                 0,  1.000000000000000, -0.795242475304584, -0.577831504240967},
    {                 0,                  0,  1.000000000000000,  1.592855170983499},
    {                 0,                  0,                  0,  1.000000000000000}}, 
    { 2.624634236385219e-01, 2.558923282408059e-01, 1.210848205583995e+02, 9.986429171171181e+00}, true);
  // clang-format on

  // call UUT
  cov.apaT(A);

  // EXPECT_EQ(expCov._isInverse, cov._isInverse);
  for (sint32 i = 0; i < 4; ++i)
  {
    EXPECT_FLOAT_EQ(expCov._d.at_unsafe(i), cov._d.at_unsafe(i));
    for (sint32 j = i; j < 4; ++j)
    {
      EXPECT_FLOAT_EQ(expCov._u.at_unsafe(i, j), cov._u.at_unsafe(i, j));
      // EXPECT_FLOAT_EQ(expCov().at_unsafe(i, j), expFullCov.at_unsafe(i, j));
    }
  }
}
#endif

TEST(CovarianceMatrixFactored, rank1Update_upper) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1,2,3}, 
    {0,1,4}, 
    {0,0,1}}, {1, 2, 4}, false);

  const auto x = tracking::math::Vector<float32, 3>::FromList({1,2,3});

  const auto expCov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1.000000000000000, 0.894009216589862, 1.588235294117647},
    {                0, 1.000000000000000, 2.235294117647059},
    {                0,                 0, 1.000000000000000}}, {3.654377880184332, 25.52941176470588, 8.5}, false);
  // clang-format on

  // call UUT
  cov.rank1Update(0.5, x);

  // verify
  EXPECT_EQ(expCov._isInverse, cov._isInverse);
  for (sint32 i = 0; i < 3; ++i)
  {
    EXPECT_FLOAT_EQ(expCov._d.at_unsafe(i), cov._d.at_unsafe(i));
    for (sint32 j = i; j < 3; ++j)
    {
      EXPECT_FLOAT_EQ(expCov._u.at_unsafe(i, j), cov._u.at_unsafe(i, j));
    }
  }
}

TEST(CovarianceMatrixFactored, rank1Update_upper_inverse) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1,2,3}, 
    {0,1,4}, 
    {0,0,1}}, {1, 2, 4}, true);

  const auto x = tracking::math::Vector<float32, 3>::FromList({3,2,1});

  const auto expCov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1.000000000000000, 0.909090909090909, 0.818181818181818}, 
    {                0, 1.000000000000000, 3.157894736842105}, 
    {                0,                 0, 1.000000000000000}}, {5.5, 3.454545454545455, 7.368421052631579}, true);
  // clang-format on

  // call UUT
  cov.rank1Update(0.5, x);

  // verify
  EXPECT_EQ(expCov._isInverse, cov._isInverse);
  for (sint32 i = 0; i < 3; ++i)
  {
    EXPECT_FLOAT_EQ(expCov._d.at_unsafe(i), cov._d.at_unsafe(i));
    for (sint32 j = i; j < 3; ++j)
    {
      EXPECT_FLOAT_EQ(expCov._u.at_unsafe(i, j), cov._u.at_unsafe(i, j));
    }
  }
}

TEST(CovarianceMatrixFactored, setVariance) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::CovarianceMatrixFactored<float32, 3>::FromList({
    {1,2,3}, 
    {0,1,4}, 
    {0,0,1}}, {1, 2, 4}, false);

  const auto expMat = tracking::math::CovarianceMatrixFull<float32, 3>::FromList({
    {2,  0,  0},
    {0, 66, 16},
    {0, 16,  4}});
  // clang-format on

  // call UUT
  cov.setVariance(0, 2);

  // verify
  const auto fullCov = cov();
  EXPECT_EQ(expMat._data, fullCov._data);
}
