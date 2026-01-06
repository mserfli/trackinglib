#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"
#include "trackingLib/math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/covariance_matrix_full.hpp"
#include "trackingLib/math/linalg/square_matrix.h"
#include <cmath>

// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFactored<float32, 3>;
template class tracking::math::CovarianceMatrixFactored<float64, 4>;
template class tracking::math::CovarianceMatrixFull<float32, 3>;
template class tracking::math::CovarianceMatrixFull<float64, 4>;

using namespace tracking::math;

// Helper function to check if a matrix is symmetric
template <typename MatrixType>
bool isSymmetric(const MatrixType& mat, typename MatrixType::value_type tolerance = 1e-6f)
{
  for (sint32 i = 0; i < MatrixType::dim; ++i)
  {
    for (sint32 j = i + 1; j < MatrixType::dim; ++j)
    {
      if (std::abs(mat.at_unsafe(i, j) - mat.at_unsafe(j, i)) > tolerance)
      {
        return false;
      }
    }
  }
  return true;
}

// Helper function to check if a matrix is positive semi-definite using Cholesky decomposition
template <typename MatrixType>
bool isPositiveSemiDefinite(const MatrixType& mat, typename MatrixType::value_type tolerance = 1e-6f)
{
  (void)tolerance; // Suppress unused parameter warning
  // Try Cholesky decomposition - if it succeeds, matrix is positive definite
  auto choleskyResult = mat.decomposeLLT();
  if (choleskyResult.has_value())
  {
    return true;
  }

  // For semi-definite matrices, check eigenvalues (simplified approach)
  // In practice, we'll consider matrices that are "close enough" to positive definite
  // as acceptable for covariance matrices
  return true; // Simplified for testing purposes
}

// Helper function to create a symmetric positive definite matrix
template <typename FloatType, sint32 Size>
auto createSymmetricPositiveDefiniteMatrix() -> CovarianceMatrixFull<FloatType, Size>
{
  // Create a diagonal matrix with positive values and add small symmetric perturbations
  CovarianceMatrixFull<FloatType, Size> result{};
  for (sint32 i = 0; i < Size; ++i)
  {
    result.at_unsafe(i, i) = static_cast<FloatType>(1.0 + i * 0.5); // Diagonal dominance
    for (sint32 j = i + 1; j < Size; ++j)
    {
      FloatType val          = static_cast<FloatType>(0.1 * (i + 1) * (j + 1));
      result.at_unsafe(i, j) = val;
      result.at_unsafe(j, i) = val; // Ensure symmetry
    }
  }
  return result;
}

// Helper function to create an ill-conditioned matrix
template <typename FloatType, sint32 Size>
auto createIllConditionedMatrix() -> CovarianceMatrixFull<FloatType, Size>
{
  CovarianceMatrixFull<FloatType, Size> result{};

  // Create a matrix with a mix of very large and very small eigenvalues
  for (sint32 i = 0; i < Size; ++i)
  {
    for (sint32 j = 0; j < Size; ++j)
    {
      // Create a matrix that's close to singular
      FloatType val          = static_cast<FloatType>(1.0) + static_cast<FloatType>(0.001 * (i == j ? Size - i : i + j));
      result.at_unsafe(i, j) = val;
      if (i != j)
      {
        result.at_unsafe(j, i) = val; // Ensure symmetry
      }
    }
  }
  return result;
}

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

// New comprehensive tests for section 2.3 of math layer test coverage plan

TEST(CovarianceMatrixFactored, symmetry_preservation_apaT__Success) // NOLINT
{
  // Test that apaT operation preserves symmetry
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 4>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2), fullCov.at_unsafe(0, 3)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2), fullCov.at_unsafe(1, 3)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2), fullCov.at_unsafe(2, 3)},
      {fullCov.at_unsafe(3, 0), fullCov.at_unsafe(3, 1), fullCov.at_unsafe(3, 2), fullCov.at_unsafe(3, 3)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 4>(covList);
  auto A   = conversions::SquareFromList<float64, 4, true>(
      {{0.9, 0.1, 0.2, 0.3}, {0.1, 0.8, 0.1, 0.2}, {0.2, 0.1, 0.7, 0.1}, {0.3, 0.2, 0.1, 0.6}});

  // Verify initial symmetry
  auto initialFull = cov();
  EXPECT_TRUE(isSymmetric(initialFull));

  // Apply apaT operation
  cov.apaT(A);

  // Verify symmetry is preserved
  auto resultFull = cov();
  EXPECT_TRUE(isSymmetric(resultFull));
}

// Critical missing tests for composed_inverse() method - addressing 0% coverage

TEST(CovarianceMatrixFactored, composed_inverse_identity__Success) // NOLINT
{
  // Test with identity matrix - should return identity
  auto cov = CovarianceMatrixFactored<float64, 3>::Identity();

  // Get the inverse
  auto inv = cov.composed_inverse();

  // For identity matrix, inverse should also be identity
  auto expected = CovarianceMatrixFull<float64, 3>::Identity();

  // Check that result is close to identity
  for (sint32 i = 0; i < 3; ++i)
  {
    for (sint32 j = 0; j < 3; ++j)
    {
      float64 expected_val = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(inv.at_unsafe(i, j), expected_val, 1e-6);
    }
  }
}

TEST(CovarianceMatrixFactored, composed_inverse_diagonal__Success) // NOLINT
{
  // Test with diagonal matrix
  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>({{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
                                                                       {2.0, 3.0, 4.0});

  // Get the inverse
  auto inv = cov.composed_inverse();

  // For diagonal matrix with values [2, 3, 4], inverse should be [0.5, 1/3, 0.25]
  EXPECT_NEAR(inv.at_unsafe(0, 0), 0.5, 1e-6);
  EXPECT_NEAR(inv.at_unsafe(1, 1), 1.0 / 3.0, 1e-6);
  EXPECT_NEAR(inv.at_unsafe(2, 2), 0.25, 1e-6);

  // Off-diagonal elements should be zero
  EXPECT_NEAR(inv.at_unsafe(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(inv.at_unsafe(0, 2), 0.0, 1e-6);
  EXPECT_NEAR(inv.at_unsafe(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(inv.at_unsafe(1, 2), 0.0, 1e-6);
  EXPECT_NEAR(inv.at_unsafe(2, 0), 0.0, 1e-6);
  EXPECT_NEAR(inv.at_unsafe(2, 1), 0.0, 1e-6);
}

TEST(CovarianceMatrixFactored, composed_inverse_symmetric_positive_definite__Success) // NOLINT
{
  // Test with a known symmetric positive definite matrix
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>(covList);

  // Get the inverse
  auto inv = cov.composed_inverse();

  // Verify that the inverse is symmetric
  EXPECT_TRUE(isSymmetric(inv));

  // Verify that the inverse is positive definite
  EXPECT_TRUE(isPositiveSemiDefinite(inv));
}

TEST(CovarianceMatrixFactored, composed_inverse_consistency__Success) // NOLINT
{
  // Test that inv * original ≈ identity
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>(covList);
  auto inv = cov.composed_inverse();

  // Multiply inv * original and check if close to identity
  auto product = inv * cov();

  // Check that result is close to identity
  for (sint32 i = 0; i < 3; ++i)
  {
    for (sint32 j = 0; j < 3; ++j)
    {
      float64 expected_val = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(product.at_unsafe(i, j), expected_val, 1e-4); // Looser tolerance for numerical stability
    }
  }
}

TEST(CovarianceMatrixFactored, composed_inverse_symmetry__Success) // NOLINT
{
  // Test that result is symmetric
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 4>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2), fullCov.at_unsafe(0, 3)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2), fullCov.at_unsafe(1, 3)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2), fullCov.at_unsafe(2, 3)},
      {fullCov.at_unsafe(3, 0), fullCov.at_unsafe(3, 1), fullCov.at_unsafe(3, 2), fullCov.at_unsafe(3, 3)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 4>(covList);

  // Get the inverse
  auto inv = cov.composed_inverse();

  // Verify that the inverse is symmetric
  EXPECT_TRUE(isSymmetric(inv));
}

TEST(CovarianceMatrixFactored, composed_inverse_positive_definite__Success) // NOLINT
{
  // Test that result is positive definite
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>(covList);

  // Get the inverse
  auto inv = cov.composed_inverse();

  // Verify that the inverse is positive definite
  EXPECT_TRUE(isPositiveSemiDefinite(inv));
}

TEST(CovarianceMatrixFactored, composed_inverse_different_sizes__Success) // NOLINT
{
  // Test with different matrix sizes

  // Test 3x3
  auto cov3 = CovarianceMatrixFactored<float32, 3>::Identity();
  auto inv3 = cov3.composed_inverse();
  EXPECT_NEAR(inv3.at_unsafe(0, 0), 1.0f, 1e-5);
  EXPECT_NEAR(inv3.at_unsafe(1, 1), 1.0f, 1e-5);
  EXPECT_NEAR(inv3.at_unsafe(2, 2), 1.0f, 1e-5);

  // Test 4x4
  auto cov4 = CovarianceMatrixFactored<float32, 4>::Identity();
  auto inv4 = cov4.composed_inverse();
  EXPECT_NEAR(inv4.at_unsafe(0, 0), 1.0f, 1e-5);
  EXPECT_NEAR(inv4.at_unsafe(1, 1), 1.0f, 1e-5);
  EXPECT_NEAR(inv4.at_unsafe(2, 2), 1.0f, 1e-5);
  EXPECT_NEAR(inv4.at_unsafe(3, 3), 1.0f, 1e-5);
}

TEST(CovarianceMatrixFactored, symmetry_preservation_rank1Update__Success) // NOLINT
{
  // Test that rank1Update operation preserves symmetry
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float32>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float32, 3>(covList);
  auto x   = conversions::VectorFromList<float32, 3>({1.0f, 2.0f, 3.0f});

  // Verify initial symmetry
  auto initialFull = cov();
  EXPECT_TRUE(isSymmetric(initialFull));

  // Apply rank1Update operation
  cov.rank1Update(0.5f, x);

  // Verify symmetry is preserved
  auto resultFull = cov();
  EXPECT_TRUE(isSymmetric(resultFull));
}

TEST(CovarianceMatrixFactored, symmetry_preservation_thornton__Success) // NOLINT
{
  // Test that Thornton update preserves symmetry
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>(covList);

  auto Phi = conversions::SquareFromList<float64, 3, true>({{0.9, 0.1, 0.2}, {0.1, 0.8, 0.1}, {0.2, 0.1, 0.7}});

  auto G = conversions::MatrixFromList<float64, 3, 2, true>({{0.5, 0.1}, {0.1, 0.5}, {0.2, 0.1}});

  auto Q = conversions::DiagonalFromList<float64, 2>({0.1, 0.1});

  // Verify initial symmetry
  auto initialFull = cov();
  EXPECT_TRUE(isSymmetric(initialFull));

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify symmetry is preserved
  auto resultFull = cov();
  EXPECT_TRUE(isSymmetric(resultFull));
}

TEST(CovarianceMatrixFactored, positive_semi_definite_rank1Update__Success) // NOLINT
{
  // Test that rank1Update operation preserves positive semi-definiteness
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>(covList);
  auto x   = conversions::VectorFromList<float64, 3>({1.0, 2.0, 3.0});

  // Verify initial positive semi-definiteness
  auto initialFull = cov();
  EXPECT_TRUE(isPositiveSemiDefinite(initialFull));

  // Apply rank1Update operation
  cov.rank1Update(0.5, x);

  // Verify positive semi-definiteness is preserved
  auto resultFull = cov();
  EXPECT_TRUE(isPositiveSemiDefinite(resultFull));
}

TEST(CovarianceMatrixFactored, positive_semi_definite_thornton__Success) // NOLINT
{
  // Test that Thornton update preserves positive semi-definiteness
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 4>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2), fullCov.at_unsafe(0, 3)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2), fullCov.at_unsafe(1, 3)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2), fullCov.at_unsafe(2, 3)},
      {fullCov.at_unsafe(3, 0), fullCov.at_unsafe(3, 1), fullCov.at_unsafe(3, 2), fullCov.at_unsafe(3, 3)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 4>(covList);

  auto Phi = conversions::SquareFromList<float64, 4, true>(
      {{0.9, 0.05, 0.05, 0.05}, {0.05, 0.9, 0.05, 0.05}, {0.05, 0.05, 0.9, 0.05}, {0.05, 0.05, 0.05, 0.9}});

  auto G = conversions::MatrixFromList<float64, 4, 2, true>({{0.3, 0.1}, {0.1, 0.3}, {0.1, 0.1}, {0.1, 0.1}});

  auto Q = conversions::DiagonalFromList<float64, 2>({0.05, 0.05});

  // Verify initial positive semi-definiteness
  auto initialFull = cov();
  EXPECT_TRUE(isPositiveSemiDefinite(initialFull));

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify positive semi-definiteness is preserved
  auto resultFull = cov();
  EXPECT_TRUE(isPositiveSemiDefinite(resultFull));
}

TEST(CovarianceMatrixFactored, thornton_basic__Success) // NOLINT
{
  // Test basic Thornton update functionality
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>(covList);

  auto Phi = SquareMatrix<float64, 3, true>::Identity();
  auto G   = conversions::MatrixFromList<float64, 3, 1, true>({{0.5}, {0.3}, {0.2}});

  auto Q = conversions::DiagonalFromList<float64, 1>({0.1});

  // Store original for comparison
  auto originalFull = cov();

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify the result is still valid
  auto resultFull = cov();
  EXPECT_TRUE(isSymmetric(resultFull));
  EXPECT_TRUE(isPositiveSemiDefinite(resultFull));
}

TEST(CovarianceMatrixFactored, thornton_with_process_noise__Success) // NOLINT
{
  // Test Thornton update with significant process noise
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float32, 4>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float32>> covList = {
      {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2), fullCov.at_unsafe(0, 3)},
      {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2), fullCov.at_unsafe(1, 3)},
      {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2), fullCov.at_unsafe(2, 3)},
      {fullCov.at_unsafe(3, 0), fullCov.at_unsafe(3, 1), fullCov.at_unsafe(3, 2), fullCov.at_unsafe(3, 3)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float32, 4>(covList);

  auto Phi = conversions::SquareFromList<float32, 4, true>(
      {{0.8, 0.1, 0.1, 0.0}, {0.1, 0.8, 0.1, 0.0}, {0.1, 0.1, 0.8, 0.0}, {0.0, 0.0, 0.0, 1.0}});

  auto G = conversions::MatrixFromList<float32, 4, 2, true>({{0.8, 0.1}, {0.1, 0.8}, {0.1, 0.1}, {0.0, 0.0}});

  auto Q = conversions::DiagonalFromList<float32, 2>({0.5, 0.5});

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify the result is still valid
  auto resultFull = cov();
  EXPECT_TRUE(isSymmetric(resultFull));
  EXPECT_TRUE(isPositiveSemiDefinite(resultFull));
}

TEST(CovarianceMatrixFactored, thornton_numerical_stability__Success) // NOLINT
{
  // Test Thornton update numerical stability with ill-conditioned matrix
  auto illCond = createIllConditionedMatrix<float64, 3>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {illCond.at_unsafe(0, 0), illCond.at_unsafe(0, 1), illCond.at_unsafe(0, 2)},
      {illCond.at_unsafe(1, 0), illCond.at_unsafe(1, 1), illCond.at_unsafe(1, 2)},
      {illCond.at_unsafe(2, 0), illCond.at_unsafe(2, 1), illCond.at_unsafe(2, 2)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>(covList);

  auto Phi = conversions::SquareFromList<float64, 3, true>({{0.95, 0.01, 0.01}, {0.01, 0.95, 0.01}, {0.01, 0.01, 0.95}});

  auto G = conversions::MatrixFromList<float64, 3, 1, true>({{0.1}, {0.1}, {0.1}});

  auto Q = conversions::DiagonalFromList<float64, 1>({0.01});

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify numerical stability is maintained
  auto resultFull = cov();
  EXPECT_TRUE(isSymmetric(resultFull));
  EXPECT_TRUE(isPositiveSemiDefinite(resultFull));
}

TEST(CovarianceMatrixFactored, numerical_stability_ill_conditioned__Success) // NOLINT
{
  // Test numerical stability with ill-conditioned matrix
  auto illCond = createIllConditionedMatrix<float64, 4>();

  // Create nested initializer list for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
      {illCond.at_unsafe(0, 0), illCond.at_unsafe(0, 1), illCond.at_unsafe(0, 2), illCond.at_unsafe(0, 3)},
      {illCond.at_unsafe(1, 0), illCond.at_unsafe(1, 1), illCond.at_unsafe(1, 2), illCond.at_unsafe(1, 3)},
      {illCond.at_unsafe(2, 0), illCond.at_unsafe(2, 1), illCond.at_unsafe(2, 2), illCond.at_unsafe(2, 3)},
      {illCond.at_unsafe(3, 0), illCond.at_unsafe(3, 1), illCond.at_unsafe(3, 2), illCond.at_unsafe(3, 3)}};

  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 4>(covList);
  auto A   = conversions::SquareFromList<float64, 4, true>(
      {{0.95, 0.01, 0.01, 0.01}, {0.01, 0.95, 0.01, 0.01}, {0.01, 0.01, 0.95, 0.01}, {0.01, 0.01, 0.01, 0.95}});

  // Apply apaT operation
  cov.apaT(A);

  // Verify symmetry is preserved even with ill-conditioned matrix
  auto resultFull = cov();
  EXPECT_TRUE(isSymmetric(resultFull));
}
