#include "gtest/gtest.h"
#include "base/atomic_types.h"
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"   // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/diagonal_matrix.hpp"                    // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix.hpp"                             // IWYU pragma: keep
#include "trackingLib/math/linalg/modified_gram_schmidt.hpp"              // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.hpp"                      // IWYU pragma: keep
#include "trackingLib/math/linalg/triangular_matrix.hpp"                  // IWYU pragma: keep
#include "trackingLib/math/linalg/vector.hpp"                             // IWYU pragma: keep

using namespace tracking::math;

/// \brief Test fixture for ModifiedGramSchmidt tests
class GTestModifiedGramSchmidt: public ::testing::Test
{
protected:
  /// \brief Tolerance for floating-point comparisons
  static constexpr float64 TOLERANCE = 1e-10;

  /// \brief Helper function to reconstruct matrix from UDU factorization
  /// \param u Unit upper triangular matrix
  /// \param d Diagonal matrix
  /// \return Reconstructed matrix U*D*U'
  template <sint32 Size_>
  auto reconstructUDU(const TriangularMatrix<float64, Size_, false, true>& u, const DiagonalMatrix<float64, Size_>& d) const
  {
    // U*D*U' = U * (D * U')
    const auto d_ut = d * u.transpose();
    return u * d_ut;
  }

  /// \brief Helper function to check if two matrices are approximately equal
  /// \param a First matrix
  /// \param b Second matrix
  /// \param tol Tolerance for comparison
  /// \return true if matrices are approximately equal
  template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_, bool IsRowMajor2_>
  bool matricesApproxEqual(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>&  a,
                           const Matrix<ValueType_, Rows_, Cols_, IsRowMajor2_>& b,
                           float64                                               tol = TOLERANCE) const
  {
    if (a.Rows != b.Rows || a.Cols != b.Cols)
    {
      return false;
    }
    for (sint32 i = 0; i < a.Rows; ++i)
    {
      for (sint32 j = 0; j < a.Cols; ++j)
      {
        if (std::abs(a.at_unsafe(i, j) - b.at_unsafe(i, j)) > tol)
        {
          return false;
        }
      }
    }
    return true;
  }
};

/// \brief Parameterized test for different matrix sizes
class GTestModifiedGramSchmidtParameterized
    : public GTestModifiedGramSchmidt
    , public ::testing::WithParamInterface<sint32>
{
};

/// \brief Test cases for different matrix sizes
INSTANTIATE_TEST_SUITE_P(MatrixSizes, GTestModifiedGramSchmidtParameterized, ::testing::Values(2, 4, 6));

/// \brief Test Phi*UDU'*Phi' transformation
TEST_P(GTestModifiedGramSchmidtParameterized, run_PhiUDUPhiT__Success) // NOLINT
{
  const sint32 size = GetParam();

  // Create test matrices based on size
  if (size == 2)
  {
    // clang-format off
    const auto phi = conversions::SquareFromList<float64, 2, true>({
        {1.0, 0.1},
        {0.0, 1.0},
    });
    auto u = conversions::TriangularFromSquare<float64, 2, false, true>(conversions::SquareFromList<float64, 2, true>({
        {1.0, 0.5},
        {0.0, 1.0},
    }));
    auto d = conversions::DiagonalFromList<float64, 2>({2.0, 3.0});
    // clang-format on

    // Expected: Phi * (U*D*U') * Phi'
    auto       p        = reconstructUDU<2>(u, d);
    const auto phi_p    = phi * p;
    const auto expected = phi_p * phi.transpose();

    // Call UUT
    ModifiedGramSchmidt<float64, 2>::run(u, d, phi);

    // Verify reconstruction
    const auto result = reconstructUDU<2>(u, d);
    EXPECT_TRUE(matricesApproxEqual(result, expected));
  }
  else if (size == 4)
  {
    // clang-format off
    const auto phi = conversions::SquareFromList<float64, 4, true>({
        {1.0, 0.0, 0.0, 0.0},
        {0.1, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0},
    });
    auto u = conversions::TriangularFromSquare<float64, 4, false, true>(conversions::SquareFromList<float64, 4, true>({
        {1.0, 0.2, 0.1, 0.0},
        {0.0, 1.0, 0.3, 0.0},
        {0.0, 0.0, 1.0, 0.4},
        {0.0, 0.0, 0.0, 1.0},
    }));
    auto d = conversions::DiagonalFromList<float64, 4>({1.0, 2.0, 3.0, 4.0});
    // clang-format on

    // Expected: Phi * (U*D*U') * Phi'
    const auto p        = reconstructUDU<4>(u, d);
    const auto phi_p    = phi * p;
    const auto expected = phi_p * phi.transpose();

    // Call UUT
    ModifiedGramSchmidt<float64, 4>::run(u, d, phi);

    // Verify reconstruction
    const auto result = reconstructUDU<4>(u, d);
    EXPECT_TRUE(matricesApproxEqual(result, expected));
  }
  else if (size == 6)
  {
    // clang-format off
    const auto phi = conversions::SquareFromList<float64, 6, true>({
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    });
    auto u = conversions::TriangularFromSquare<float64, 6, false, true>(conversions::SquareFromList<float64, 6, true>({
        {1.0, 0.1, 0.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.2, 0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.3, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0, 0.4, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.5},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    }));
    auto d = conversions::DiagonalFromList<float64, 6>({1.0, 1.5, 2.0, 2.5, 3.0, 3.5});
    // clang-format on

    // Expected: Phi * (U*D*U') * Phi'
    const auto p        = reconstructUDU<6>(u, d);
    const auto phi_p    = phi * p;
    const auto expected = phi_p * phi.transpose();

    // Call UUT
    ModifiedGramSchmidt<float64, 6>::run(u, d, phi);

    // Verify reconstruction
    const auto result = reconstructUDU<6>(u, d);
    EXPECT_TRUE(matricesApproxEqual(result, expected));
  }
}

/// \brief Test Phi*UDU'*Phi' + G*Q*G' transformation
TEST_P(GTestModifiedGramSchmidtParameterized, run_PhiUDUPhiT_PlusGQGT__Success) // NOLINT
{
  const sint32 size = GetParam();

  if (size == 2)
  {
    // clang-format off
    const auto phi = conversions::SquareFromList<float64, 2, true>({
        {1.0, 0.1},
        {0.0, 1.0},
    });
    auto u = conversions::TriangularFromSquare<float64, 2, false, true>(conversions::SquareFromList<float64, 2, true>({
        {1.0, 0.5},
        {0.0, 1.0},
    }));
    auto d = conversions::DiagonalFromList<float64, 2>({2.0, 3.0});
    const auto g = conversions::MatrixFromList<float64, 2, 1, true>({
        {0.1},
        {0.2},
    });
    const auto q = conversions::DiagonalFromList<float64, 1>({0.5});
    // clang-format on

    // Expected: Phi * (U*D*U') * Phi' + G*Q*G'
    const auto p           = reconstructUDU<2>(u, d);
    const auto phi_p       = phi * p;
    const auto phi_p_phi_t = phi_p * phi.transpose();
    const auto g_q         = g * q;
    const auto g_q_gt      = g_q * g.transpose();
    const auto expected    = phi_p_phi_t + g_q_gt;

    // Call UUT
    ModifiedGramSchmidt<float64, 2>::run<1>(u, d, phi, g, q);

    // Verify reconstruction
    const auto result = reconstructUDU<2>(u, d);
    EXPECT_TRUE(matricesApproxEqual(result, expected));
  }
  else if (size == 4)
  {
    // clang-format off
    const auto phi = conversions::SquareFromList<float64, 4, true>({
        {1.0, 0.0, 0.0, 0.0},
        {0.1, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0},
    });
    auto u = conversions::TriangularFromSquare<float64, 4, false, true>(conversions::SquareFromList<float64, 4, true>({
        {1.0, 0.2, 0.1, 0.0},
        {0.0, 1.0, 0.3, 0.0},
        {0.0, 0.0, 1.0, 0.4},
        {0.0, 0.0, 0.0, 1.0},
    }));
    auto d = conversions::DiagonalFromList<float64, 4>({1.0, 2.0, 3.0, 4.0});
    const auto g = conversions::MatrixFromList<float64, 4, 2, true>({
        {0.1, 0.0},
        {0.0, 0.1},
        {0.2, 0.0},
        {0.0, 0.2},
    });
    const auto q = conversions::DiagonalFromList<float64, 2>({0.5, 0.8});
    // clang-format on

    // Expected: Phi * (U*D*U') * Phi' + G*Q*G'
    const auto p           = reconstructUDU<4>(u, d);
    const auto phi_p       = phi * p;
    const auto phi_p_phi_t = phi_p * phi.transpose();
    const auto g_q         = g * q;
    const auto g_q_gt      = g_q * g.transpose();
    const auto expected    = phi_p_phi_t + g_q_gt;

    // Call UUT
    ModifiedGramSchmidt<float64, 4>::run<2>(u, d, phi, g, q);

    // Verify reconstruction
    const auto result = reconstructUDU<4>(u, d);
    EXPECT_TRUE(matricesApproxEqual(result, expected));
  }
  else if (size == 6)
  {
    // clang-format off
    const auto phi = conversions::SquareFromList<float64, 6, true>({
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    });
    auto u = conversions::TriangularFromSquare<float64, 6, false, true>(conversions::SquareFromList<float64, 6, true>({
        {1.0, 0.1, 0.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.2, 0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.3, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0, 0.4, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.5},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    }));
    auto d = conversions::DiagonalFromList<float64, 6>({1.0, 1.5, 2.0, 2.5, 3.0, 3.5});
    const auto g = conversions::MatrixFromList<float64, 6, 3, true>({
        {0.1, 0.0, 0.0},
        {0.0, 0.1, 0.0},
        {0.0, 0.0, 0.1},
        {0.2, 0.0, 0.0},
        {0.0, 0.2, 0.0},
        {0.0, 0.0, 0.2},
    });
    const auto q = conversions::DiagonalFromList<float64, 3>({0.5, 0.8, 1.2});
    // clang-format on

    // Expected: Phi * (U*D*U') * Phi' + G*Q*G'
    const auto p           = reconstructUDU<6>(u, d);
    const auto phi_p       = phi * p;
    const auto phi_p_phi_t = phi_p * phi.transpose();
    const auto g_q         = g * q;
    const auto g_q_gt      = g_q * g.transpose();
    const auto expected    = phi_p_phi_t + g_q_gt;

    // Call UUT
    ModifiedGramSchmidt<float64, 6>::run<3>(u, d, phi, g, q);

    // Verify reconstruction
    const auto result = reconstructUDU<6>(u, d);
    EXPECT_TRUE(matricesApproxEqual(result, expected));
  }
}

/// \brief Test numerical stability with ill-conditioned matrices
TEST_F(GTestModifiedGramSchmidt, run_NumericalStability__Success) // NOLINT
{
  // Use a matrix with high condition number
  // clang-format off
  const auto phi = conversions::SquareFromList<float64, 4, true>({
      {1.0, 1e-8, 0.0, 0.0},
      {0.0, 1.0, 1e-8, 0.0},
      {0.0, 0.0, 1.0, 1e-8},
      {0.0, 0.0, 0.0, 1.0},
  });
  auto u = conversions::TriangularFromSquare<float64, 4, false, true>(conversions::SquareFromList<float64, 4, true>({
      {1.0, 0.1, 0.0, 0.0},
      {0.0, 1.0, 0.2, 0.0},
      {0.0, 0.0, 1.0, 0.3},
      {0.0, 0.0, 0.0, 1.0},
  }));
  auto d = conversions::DiagonalFromList<float64, 4>({1e-10, 1e-5, 1.0, 1e5});
  // clang-format on

  // Expected: Phi * (U*D*U') * Phi'
  auto       p        = reconstructUDU<4>(u, d);
  const auto phi_p    = phi * p;
  const auto expected = phi_p * phi.transpose();

  // Call UUT
  ModifiedGramSchmidt<float64, 4>::run(u, d, phi);

  // Verify reconstruction with relaxed tolerance for numerical stability
  const auto result = reconstructUDU<4>(u, d);
  EXPECT_TRUE(matricesApproxEqual(result, expected, 1e-6));
}

/// \brief Test with identity Phi matrix
TEST_F(GTestModifiedGramSchmidt, run_IdentityPhi__Success) // NOLINT
{
  // clang-format off
  const auto phi = conversions::SquareFromList<float64, 3, true>({
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  });
  auto u = conversions::TriangularFromSquare<float64, 3, false, true>(conversions::SquareFromList<float64, 3, true>({
      {1.0, 0.5, 0.2},
      {0.0, 1.0, 0.3},
      {0.0, 0.0, 1.0},
  }));
  auto d = conversions::DiagonalFromList<float64, 3>({2.0, 3.0, 4.0});
  // clang-format on

  // Expected: Phi * (U*D*U') * Phi' = U*D*U' since Phi = I
  const auto expected = reconstructUDU<3>(u, d);

  // Call UUT
  ModifiedGramSchmidt<float64, 3>::run(u, d, phi);

  // Verify reconstruction
  const auto result = reconstructUDU<3>(u, d);
  EXPECT_TRUE(matricesApproxEqual(result, expected));
}

/// \brief Test with zero process noise
TEST_F(GTestModifiedGramSchmidt, run_ZeroProcessNoise__Success) // NOLINT
{
  // clang-format off
  const auto phi = conversions::SquareFromList<float64, 2, true>({
      {1.0, 0.1},
      {0.0, 1.0},
  });
  auto u = conversions::TriangularFromSquare<float64, 2, false, true>(conversions::SquareFromList<float64, 2, true>({
      {1.0, 0.5},
      {0.0, 1.0},
  }));
  auto d = conversions::DiagonalFromList<float64, 2>({2.0, 3.0});
  const auto g = conversions::MatrixFromList<float64, 2, 1, true>({
      {0.1},
      {0.2},
  });
  const auto q = conversions::DiagonalFromList<float64, 1>({0.0}); // Zero noise
  // clang-format on

  // Expected: Phi * (U*D*U') * Phi' + G*Q*G' = Phi * (U*D*U') * Phi' since Q=0
  const auto p        = reconstructUDU<2>(u, d);
  const auto phi_p    = phi * p;
  const auto expected = phi_p * phi.transpose();

  // Call UUT
  ModifiedGramSchmidt<float64, 2>::run<1>(u, d, phi, g, q);

  // Verify reconstruction
  const auto result = reconstructUDU<2>(u, d);
  EXPECT_TRUE(matricesApproxEqual(result, expected));
}
