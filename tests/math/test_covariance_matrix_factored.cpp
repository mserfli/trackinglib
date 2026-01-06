#include "gtest/gtest.h"
#include "math/linalg/covariance_matrix_factored.h"
#include "trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"
#include "trackingLib/math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.h"
#include <cmath>

// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFactored<float32, 3>;
template class tracking::math::CovarianceMatrixFactored<float64, 4>;
template class tracking::math::CovarianceMatrixFull<float32, 3>;
template class tracking::math::CovarianceMatrixFull<float64, 4>;

using namespace tracking::math;

// Helper function to create a factored symmetric positive definite matrix
template <typename FloatType, sint32 Size>
auto createFactoredSymmetricPositiveDefiniteMatrix() -> CovarianceMatrixFactored<FloatType, Size>
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
  auto covFactored = conversions::CovarianceMatrixFactoredFromCovarianceMatrixFull<FloatType, Size>(result);
  testing::AssertionResult(covFactored.has_value()) << "Failed to factor symmetric positive definite matrix into UDUt form.";
  return covFactored.value();
}

// Helper function to create a factored ill-conditioned matrix
template <typename FloatType, sint32 Size>
auto createFactoredIllConditionedMatrix() -> CovarianceMatrixFactored<FloatType, Size>
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
  auto covFactored = conversions::CovarianceMatrixFactoredFromCovarianceMatrixFull<FloatType, Size>(result);
  testing::AssertionResult(covFactored.has_value()) << "Failed to factor symmetric positive definite matrix into UDUt form.";
  return covFactored.value();
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

TEST(CovarianceMatrixFactored, symmetry_preservation_apaT__Success) // NOLINT
{
  // Test that apaT operation preserves symmetry
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 4>();

  // clang-format off
  auto A = conversions::SquareFromList<float64, 4, true>({
    {0.9, 0.1, 0.2, 0.3}, 
    {0.1, 0.8, 0.1, 0.2}, 
    {0.2, 0.1, 0.7, 0.1}, 
    {0.3, 0.2, 0.1, 0.6}
  });
  // clang-format on

  // Verify initial symmetry
  auto initialFull = cov();
  EXPECT_TRUE(initialFull.isSymmetric());

  // Apply apaT operation
  cov.apaT(A);

  // Verify symmetry is preserved
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isSymmetric());
}

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
  // clang-format off
  auto cov = conversions::CovarianceMatrixFactoredFromList<float64, 3>(
    {
    {1.0, 0.0, 0.0}, 
    {0.0, 1.0, 0.0}, 
    {0.0, 0.0, 1.0}
    },
    {2.0, 3.0, 4.0});
  // clang-format on

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
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 3>();

  // Get the inverse
  auto inv = cov.composed_inverse();

  // Verify that the inverse is symmetric
  EXPECT_TRUE(inv.isSymmetric());

  // Verify that the inverse is positive definite
  EXPECT_TRUE(inv.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFactored, composed_inverse_consistency__Success) // NOLINT
{
  // Test that inv * original ≈ identity
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 3>();
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
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 4>();

  // Get the inverse
  auto inv = cov.composed_inverse();

  // Verify that the inverse is symmetric
  EXPECT_TRUE(inv.isSymmetric());
}

TEST(CovarianceMatrixFactored, composed_inverse_positive_definite__Success) // NOLINT
{
  // Test that result is positive definite
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 3>();

  // Get the inverse
  auto inv = cov.composed_inverse();

  // Verify that the inverse is positive definite
  EXPECT_TRUE(inv.isPositiveSemiDefinite());
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
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float32, 3>();
  auto x   = conversions::VectorFromList<float32, 3>({1.0f, 2.0f, 3.0f});

  // Verify initial symmetry
  auto initialFull = cov();
  EXPECT_TRUE(initialFull.isSymmetric());

  // Apply rank1Update operation
  cov.rank1Update(0.5f, x);

  // Verify symmetry is preserved
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isSymmetric());
}

TEST(CovarianceMatrixFactored, symmetry_preservation_thornton__Success) // NOLINT
{
  // Test that Thornton update preserves symmetry
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 3>();

  // clang-format off
  auto Phi = conversions::SquareFromList<float64, 3, true>({
    {0.9, 0.1, 0.2}, 
    {0.1, 0.8, 0.1}, 
    {0.2, 0.1, 0.7}
  });

  auto G = conversions::MatrixFromList<float64, 3, 2, true>({
    {0.5, 0.1}, 
    {0.1, 0.5}, 
    {0.2, 0.1}
  });

  auto Q = conversions::DiagonalFromList<float64, 2>(
    {0.1, 0.1}
  );
  // clang-format on

  // Verify initial symmetry
  auto initialFull = cov();
  EXPECT_TRUE(initialFull.isSymmetric());

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify symmetry is preserved
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isSymmetric());
}

TEST(CovarianceMatrixFactored, positive_semi_definite_rank1Update__Success) // NOLINT
{
  // Test that rank1Update operation preserves positive semi-definiteness
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 3>();
  auto x   = conversions::VectorFromList<float64, 3>({1.0, 2.0, 3.0});

  // Verify initial positive semi-definiteness
  auto initialFull = cov();
  EXPECT_TRUE(initialFull.isPositiveSemiDefinite());

  // Apply rank1Update operation
  cov.rank1Update(0.5, x);

  // Verify positive semi-definiteness is preserved
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFactored, positive_semi_definite_thornton__Success) // NOLINT
{
  // Test that Thornton update preserves positive semi-definiteness
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 4>();

  // clang-format off
  auto Phi = conversions::SquareFromList<float64, 4, true>({
    {0.9, 0.05, 0.05, 0.05}, 
    {0.05, 0.9, 0.05, 0.05}, 
    {0.05, 0.05, 0.9, 0.05}, 
    {0.05, 0.05, 0.05, 0.9}
  });

  auto G = conversions::MatrixFromList<float64, 4, 2, true>({
    {0.3, 0.1}, 
    {0.1, 0.3}, 
    {0.1, 0.1}, 
    {0.1, 0.1}
  });

  auto Q = conversions::DiagonalFromList<float64, 2>(
    {0.05, 0.05}
  );
  // clang-format on

  // Verify initial positive semi-definiteness
  auto initialFull = cov();
  EXPECT_TRUE(initialFull.isPositiveSemiDefinite());

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify positive semi-definiteness is preserved
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFactored, thornton_basic__Success) // NOLINT
{
  // Test basic Thornton update functionality
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 3>();

  // clang-format off
  auto Phi = SquareMatrix<float64, 3, true>::Identity();
  auto G   = conversions::MatrixFromList<float64, 3, 1, true>({
    {0.5}, 
    {0.3}, 
    {0.2}
  });

  auto Q = conversions::DiagonalFromList<float64, 1>(
    {0.1}
  );
  // clang-format on

  // Store original for comparison
  auto originalFull = cov();

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify the result is still valid
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isSymmetric());
  EXPECT_TRUE(resultFull.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFactored, thornton_with_process_noise__Success) // NOLINT
{
  // Test Thornton update with significant process noise
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float32, 4>();

  // clang-format off
  auto Phi = conversions::SquareFromList<float32, 4, true>({
    {0.8, 0.1, 0.1, 0.0}, 
    {0.1, 0.8, 0.1, 0.0}, 
    {0.1, 0.1, 0.8, 0.0}, 
    {0.0, 0.0, 0.0, 1.0}
  });

  auto G = conversions::MatrixFromList<float32, 4, 2, true>({
    {0.8, 0.1}, 
    {0.1, 0.8}, 
    {0.1, 0.1}, 
    {0.0, 0.0}
  });

  auto Q = conversions::DiagonalFromList<float32, 2>(
    {0.5, 0.5}
  );
  // clang-format on

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify the result is still valid
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isSymmetric());
  EXPECT_TRUE(resultFull.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFactored, thornton_numerical_stability__Success) // NOLINT
{
  // Test Thornton update numerical stability with ill-conditioned matrix
  auto cov = createFactoredIllConditionedMatrix<float64, 3>();
  // clang-format off
  auto Phi = conversions::SquareFromList<float64, 3, true>({
    {0.95, 0.01, 0.01}, 
    {0.01, 0.95, 0.01}, 
    {0.01, 0.01, 0.95}
  });

  auto G = conversions::MatrixFromList<float64, 3, 1, true>({
    {0.1}, 
    {0.1}, 
    {0.1}
  });

  auto Q = conversions::DiagonalFromList<float64, 1>(
    {0.01}
  );
  // clang-format on

  // Apply Thornton update
  cov.thornton(Phi, G, Q);

  // Verify numerical stability is maintained
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isSymmetric());
  EXPECT_TRUE(resultFull.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFactored, numerical_stability_ill_conditioned__Success) // NOLINT
{
  // Test numerical stability with ill-conditioned matrix
  auto cov = createFactoredIllConditionedMatrix<float64, 4>();
  // clang-format off
  auto A = conversions::SquareFromList<float64, 4, true>({
    {0.95, 0.01, 0.01, 0.01}, 
    {0.01, 0.95, 0.01, 0.01}, 
    {0.01, 0.01, 0.95, 0.01}, 
    {0.01, 0.01, 0.01, 0.95}
  });
  // clang-format on

  // Apply apaT operation
  cov.apaT(A);

  // Verify symmetry is preserved even with ill-conditioned matrix
  auto resultFull = cov();
  EXPECT_TRUE(resultFull.isSymmetric());
}

TEST(CovarianceMatrixFactored, operator_call_const_double4__Success) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 4>();

  // Test operator()(int, int) const for all elements
  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = 0; j < 4; ++j)
    {
      auto result = cov(i, j);
      ASSERT_TRUE(result.has_value());
      EXPECT_NEAR(result.value(), cov.at_unsafe(i, j), 1e-6);
    }
  }
}

TEST(CovarianceMatrixFactored, operator_call_const_double4_diagonal__Success) // NOLINT
{
  // Test diagonal elements specifically
  auto cov = CovarianceMatrixFactored<float64, 4>::Identity();

  // Test operator()(int, int) const for diagonal elements
  for (sint32 i = 0; i < 4; ++i)
  {
    auto result = cov(i, i);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 1.0, 1e-6);
  }
}

TEST(CovarianceMatrixFactored, operator_call_const_double4_off_diagonal__Success) // NOLINT
{
  // Create a factored covariance matrix with known off-diagonal values
  auto cov     = createFactoredSymmetricPositiveDefiniteMatrix<float64, 4>();
  auto fullCov = cov();

  // Test operator()(int, int) const for off-diagonal elements
  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = i + 1; j < 4; ++j)
    {
      auto result = cov(i, j);
      ASSERT_TRUE(result.has_value());
      EXPECT_NEAR(result.value(), fullCov.at_unsafe(i, j), 1e-6);

      // Verify symmetry
      auto result_sym = cov(j, i);
      ASSERT_TRUE(result_sym.has_value());
      EXPECT_NEAR(result_sym.value(), fullCov.at_unsafe(j, i), 1e-6);
      EXPECT_NEAR(result.value(), result_sym.value(), 1e-6);
    }
  }
}

TEST(CovarianceMatrixFactored, setDiagonal_double4__Success) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = CovarianceMatrixFactored<float64, 4>::Identity();

  // Test setDiagonal()
  cov.setDiagonal(1, 2.5);

  // Verify the diagonal element is set correctly
  auto result = cov(1, 1);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 2.5, 1e-6);

  // Verify symmetry is maintained
  auto fullCov = cov();
  EXPECT_TRUE(fullCov.isSymmetric());
}

TEST(CovarianceMatrixFactored, setVariance_double4__Success) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = CovarianceMatrixFactored<float64, 4>::Identity();

  // Test setVariance()
  cov.setVariance(2, 3.7);

  // Verify the variance is set correctly
  auto result = cov(2, 2);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 3.7, 1e-6);

  // Verify correlations are cleared (diagonal should be set, off-diagonal should be zero for that row/col)
  auto fullCov = cov();
  EXPECT_NEAR(fullCov.at_unsafe(2, 2), 3.7, 1e-6);
  EXPECT_NEAR(fullCov.at_unsafe(2, 0), 0.0, 1e-6);
  EXPECT_NEAR(fullCov.at_unsafe(2, 1), 0.0, 1e-6);
  EXPECT_NEAR(fullCov.at_unsafe(2, 3), 0.0, 1e-6);
}

TEST(CovarianceMatrixFactored, setDiagonal_setVariance_consistency__Success) // NOLINT
{
  // Test consistency between setDiagonal and setVariance
  auto cov1 = CovarianceMatrixFactored<float64, 4>::Identity();
  auto cov2 = CovarianceMatrixFactored<float64, 4>::Identity();

  // Use setDiagonal on first
  cov1.setDiagonal(1, 2.5);

  // Use setVariance on second (should give same result for diagonal element)
  cov2.setVariance(1, 2.5);

  // Verify diagonal elements are the same
  auto result1 = cov1(1, 1);
  auto result2 = cov2(1, 1);
  ASSERT_TRUE(result1.has_value());
  ASSERT_TRUE(result2.has_value());
  EXPECT_NEAR(result1.value(), result2.value(), 1e-6);
}

TEST(CovarianceMatrixFactored, Identity_double4__Success) // NOLINT
{
  // Test Identity() method for double, 4x4
  auto cov = CovarianceMatrixFactored<float64, 4>::Identity();

  // Verify it's an identity matrix
  auto fullCov = cov();
  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = 0; j < 4; ++j)
    {
      float64 expected = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(fullCov.at_unsafe(i, j), expected, 1e-6);
    }
  }
}

TEST(CovarianceMatrixFactored, setIdentity_double4__Success) // NOLINT
{
  // Create a non-identity factored covariance matrix
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 4>();

  // Apply setIdentity()
  cov.setIdentity();

  // Verify it's now an identity matrix
  auto result = cov();
  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = 0; j < 4; ++j)
    {
      float64 expected = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(result.at_unsafe(i, j), expected, 1e-6);
    }
  }
}

TEST(CovarianceMatrixFactored, Identity_setIdentity_equivalence__Success) // NOLINT
{
  // Test equivalence between Identity() and setIdentity()
  auto cov1 = CovarianceMatrixFactored<float64, 4>::Identity();

  // Create a non-identity matrix and apply setIdentity
  auto cov2 = createFactoredSymmetricPositiveDefiniteMatrix<float64, 4>();
  cov2.setIdentity();

  // Verify both result in identity matrices
  auto result1 = cov1();
  auto result2 = cov2();

  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = 0; j < 4; ++j)
    {
      EXPECT_NEAR(result1.at_unsafe(i, j), result2.at_unsafe(i, j), 1e-6);
    }
  }
}

TEST(CovarianceMatrixFactored, at_unsafe_float6__Success) // NOLINT
{
  // Create a factored covariance matrix for float32, 6x6
  auto cov     = createFactoredSymmetricPositiveDefiniteMatrix<float32, 6>();
  auto fullCov = cov();

  // Test at_unsafe() for all elements
  for (sint32 i = 0; i < 6; ++i)
  {
    for (sint32 j = 0; j < 6; ++j)
    {
      float32 result = cov.at_unsafe(i, j);
      EXPECT_NEAR(result, fullCov.at_unsafe(i, j), 1e-5);
    }
  }
}

TEST(CovarianceMatrixFactored, at_unsafe_consistency__Success) // NOLINT
{
  // Test consistency between at_unsafe() and operator()() const
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float32, 6>();

  // Test consistency for all elements
  for (sint32 i = 0; i < 6; ++i)
  {
    for (sint32 j = 0; j < 6; ++j)
    {
      float32 unsafe_result = cov.at_unsafe(i, j);
      auto    safe_result   = cov(i, j);

      ASSERT_TRUE(safe_result.has_value());
      EXPECT_NEAR(unsafe_result, safe_result.value(), 1e-5);
    }
  }
}

// Error Handling Tests for Covariance Matrix Functions Returning tl::unexpected

TEST(CovarianceMatrixFactored, operator_call_InvalidRowIndex_ExpectError) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = CovarianceMatrixFactored<float32, 3>::Identity();

  // Test operator()(int, int) const with invalid row index < 0
  auto result = cov(-1, 0);
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, operator_call_InvalidRowIndexTooLarge_ExpectError) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = CovarianceMatrixFactored<float64, 4>::Identity();

  // Test operator()(int, int) const with invalid row index >= Size_
  auto result = cov(4, 0); // Size_ is 4, so index 4 is out of bounds
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, operator_call_InvalidColIndex_ExpectError) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = CovarianceMatrixFactored<float32, 3>::Identity();

  // Test operator()(int, int) const with invalid column index < 0
  auto result = cov(0, -1);
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, operator_call_InvalidColIndexTooLarge_ExpectError) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = CovarianceMatrixFactored<float64, 4>::Identity();

  // Test operator()(int, int) const with invalid column index >= Size_
  auto result = cov(0, 4); // Size_ is 4, so index 4 is out of bounds
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, operator_call_NegativeIndices_ExpectError) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = CovarianceMatrixFactored<float32, 3>::Identity();

  // Test operator()(int, int) const with both indices negative
  auto result = cov(-1, -1);
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, operator_call_OutOfBoundsIndices_ExpectError) // NOLINT
{
  // Create a factored covariance matrix
  auto cov = CovarianceMatrixFactored<float64, 4>::Identity();

  // Test operator()(int, int) const with both indices too large
  auto result = cov(4, 4); // Size_ is 4, so index 4 is out of bounds
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, inverse_NotPositiveDefinite_ExpectError) // NOLINT
{
  // Create a non-positive definite matrix (negative eigenvalue) directly
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float32, 3>();
  // Negative diagonal element makes it non-positive definite
  cov._d.at_unsafe(1) = -1.0f;

  // Test inverse() - should return error
  auto result = cov.inverse();
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, inverse_SingularMatrix_ExpectError) // NOLINT
{
  // Create a singular matrix (determinant = 0) directly
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float32, 3>();
  // Zero diagonal element makes it singular
  cov._d.at_unsafe(2) = 0.0f;

  // Test inverse() - should return error
  auto result = cov.inverse();
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, inverse_ZeroMatrix_ExpectError) // NOLINT
{
  // Create a zero matrix<
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 3>();
  // Set all diagonal elements to zero
  cov._d.at_unsafe(0) = 0.0;
  cov._d.at_unsafe(1) = 0.0;
  cov._d.at_unsafe(2) = 0.0;

  // Test inverse() - should return error
  auto result = cov.inverse();
  EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFactored, inverse_NegativeDefiniteMatrix_ExpectError) // NOLINT
{
  // Create a negative definite matrix
  auto cov = createFactoredSymmetricPositiveDefiniteMatrix<float64, 3>();
  // all diagonal elements negative
  cov._u.setIdentity();
  cov._d.at_unsafe(0) = -1.0;
  cov._d.at_unsafe(1) = -2.0;
  cov._d.at_unsafe(2) = -3.0;

  // Test inverse() - should return error
  auto result = cov.inverse();
  EXPECT_FALSE(result.has_value());
}
