#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"
#include "trackingLib/math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep

// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFactored<float32, 3>;

using namespace tracking::math;

TEST(CovarianceMatrixFactored, ctor_from_full_matrix) // NOLINT
{
  // clang-format off
  const auto expCov = conversions::CovarianceMatrixFactoredFromList<float32, 3>({
    {1,2,3},
    {0,1,4},
    {0,0,1}}, {1, 2, 4});

  // call UUT
  const auto cov = conversions::CovarianceMatrixFactoredFromList<float32, 3>({
    {45, 52, 12},
    {52, 66, 16},
    {12, 16,  4}});
  // clang-format on

  EXPECT_EQ(expCov._u._data, cov._u._data);
  EXPECT_EQ(expCov._d._data, cov._d._data);
}

TEST(CovarianceMatrixFactored, compose) // NOLINT
{
  // clang-format off
  auto cov = conversions::CovarianceMatrixFactoredFromList<float32, 3>({
    {1,2,3},
    {0,1,4},
    {0,0,1}}, {1, 2, 4});

  const auto expMat = conversions::CovarianceMatrixFullFromList<float32, 3>({
    {45, 52, 12},
    {52, 66, 16},
    {12, 16,  4}});
  // clang-format on

  // call UUT
  const auto composed = cov();

  EXPECT_EQ(expMat._data, composed._data);
}

struct CovarianceMatrixFactoredWithParams: public testing::TestWithParam<std::tuple<int, int>>
{
  // clang-format off
  CovarianceMatrixFactored<float32, 3> _cov = conversions::CovarianceMatrixFactoredFromList<float32, 3>({
    {1, 2, 3},
    {0, 1, 4},
    {0, 0, 1}}, {1, 2, 4});

  CovarianceMatrixFull<float32, 3> _covFull{_cov()};
  // clang-format on
};

TEST_P(CovarianceMatrixFactoredWithParams, calcCovarianceElement) // NOLINT
{
  const auto [row, col] = GetParam(); // structured binding since C++17

  // call UUT
  auto res = _cov(row, col);

  EXPECT_EQ(res, _covFull(row, col));
}

INSTANTIATE_TEST_SUITE_P(CovarianceMatrixFactored,
                         CovarianceMatrixFactoredWithParams,
                         ::testing::Combine(::testing::Range(0, 3), ::testing::Range(0, 3)));

TEST(CovarianceMatrixFactored, apaT) // NOLINT
{
  // clang-format off
  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 4>({
    {1.000000000000000,   0.378176125484376,   1.684500252612056,  -2.187010479706283},
    {                0,   1.000000000000000,  -1.241186420682381,   0.272017178830240},
    {                0,                   0,   1.000000000000000,  -0.986409098619786},
    {                0,                   0,                   0,   1.000000000000000}},
    { 0.692626242247276,   0.639133727238215,   0.839636056501929,   0.821337656508534});

  const auto A = conversions::SquareFromList<float64, 4, true>({
    {6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
    {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
    {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
    {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
   

  const auto expCov = conversions::CovarianceMatrixFactoredFromList<float64, 4>({
    {1.000000000000000,  -0.117162650062257,   0.236910463595170,   0.610508812057989},
    {                0,   1.000000000000000,   1.005949148623783,  -0.340655418792931},
    {                0,                   0,   1.000000000000000,   0.056511133324051},
    {                0,                   0,                   0,   1.000000000000000}},
    { 0.002976797231001,   0.056412527384831,   0.944727610657223,   7.233518281911786});
  // clang-format on

  // call UUT
  cov.apaT(A);

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
  const auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 4>({
    {1.000000000000000,   0.378176125484376,   1.684500252612056,  -2.187010479706283},
    {                0,   1.000000000000000,  -1.241186420682381,   0.272017178830240},
    {                0,                   0,   1.000000000000000,  -0.986409098619786},
    {                0,                   0,                   0,   1.000000000000000}},
    { 0.692626242247276,   0.639133727238215,   0.839636056501929,   0.821337656508534});

  const auto A = conversions::SquareFromList<float64, 4, true>({
    {6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
    {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
    {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
    {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}});
   

  const auto expCov = conversions::CovarianceMatrixFactoredFromList<float64, 4>({
    {1.000000000000000,  -0.117162650062257,   0.236910463595170,   0.610508812057989},
    {                0,   1.000000000000000,   1.005949148623783,  -0.340655418792931},
    {                0,                   0,   1.000000000000000,   0.056511133324051},
    {                0,                   0,                   0,   1.000000000000000}},
    { 0.002976797231001,   0.056412527384831,   0.944727610657223,   7.233518281911786});
  // clang-format on

  // call UUT
  auto res = cov.apaT(A);

  for (sint32 i = 0; i < 4; ++i)
  {
    EXPECT_FLOAT_EQ(expCov._d.at_unsafe(i), res._d.at_unsafe(i));
    for (sint32 j = i; j < 4; ++j)
    {
      EXPECT_FLOAT_EQ(expCov._u.at_unsafe(i, j), res._u.at_unsafe(i, j));
    }
  }
}
TEST(CovarianceMatrixFactored, rank1Update_upper) // NOLINT
{
  // clang-format off
  auto cov = conversions::CovarianceMatrixFactoredFromList<float32, 3>({
    {1,2,3},
    {0,1,4},
    {0,0,1}}, {1, 2, 4});

  const auto x = conversions::VectorFromList<float32, 3>({1,2,3});

  const auto expCov = conversions::CovarianceMatrixFactoredFromList<float32, 3>({
    {1.000000000000000, 0.894009216589862, 1.588235294117647},
    {                0, 1.000000000000000, 2.235294117647059},
    {                0,                 0, 1.000000000000000}}, {3.654377880184332, 25.52941176470588, 8.5});
  // clang-format on

  // call UUT
  cov.rank1Update(0.5, x);

  // verify
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
  auto cov = conversions::CovarianceMatrixFactoredFromList<float32, 3>({
    {1,2,3},
    {0,1,4},
    {0,0,1}}, {1, 2, 4});

  const auto expMat = conversions::CovarianceMatrixFullFromList<float32, 3>({
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
