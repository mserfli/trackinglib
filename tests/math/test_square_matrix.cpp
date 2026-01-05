#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.hpp"                  // IWYU pragma: keep

using namespace tracking::math;

TEST(SquareMatrix, householderQR) // NOLINT
{
  // Create a square matrix for testing
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  // clang-format on

  // call UUT
  const auto [Q, R] = mat.householderQR();

  // Check if QR is a valid decomposition by comparing to the original matrix
  const auto recomposed = Q * R;
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_FLOAT_EQ(mat.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrix, inverse) // NOLINT
{
  // Create a square matrix for testing
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  const auto expInvMat = conversions::SquareFromList<float32, 3, true>({
    {1.0,  2.0,  3.0},
    {2.0,  6.0, 14.0},
    {3.0, 14.0, 45.0}
  });
  // clang-format on

  // call UUT
  const auto inv = mat.inverse();

  // compare to expected inverse
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_NEAR(inv.at_unsafe(row, col), expInvMat.at_unsafe(row, col), 7e-5);
    }
  }
}

TEST(SquareMatrix, decomposeLLT) // NOLINT
{
  // clang-format off
  auto cov = conversions::SquareFromList<float32, 6, true>({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_TRUE(retVal.has_value());
  const auto& L = retVal.value();

  const auto recomposed = L * L.transpose();
  for (auto row = 0; row < 6; row++)
  {
    for (auto col = 0; col < 6; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrix, decomposeLLT_NotSymmetric_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = conversions::SquareFromList<float32, 2, true>({
    {10, -4},
    {-3, 13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrix, decomposeLLT_SymmetricNotPositiveDefinite_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = conversions::SquareFromList<float32, 2, true>({
    {10, -3},
    {-3, -13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrix, symmetrize) // NOLINT
{
  // clang-format off
  auto mat = conversions::SquareFromList<float32, 3, true>({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  });
  // clang-format on

  // call UUT
  mat.symmetrize();

  EXPECT_TRUE(mat.isSymmetric());

  // Check values: (A + A^T) / 2
  // (2+4)/2 = 3
  // (3+7)/2 = 5
  // (6+8)/2 = 7
  EXPECT_FLOAT_EQ(mat.at_unsafe(0, 1), 3.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(1, 0), 3.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(0, 2), 5.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(2, 0), 5.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(1, 2), 7.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(2, 1), 7.0F);
}

TEST(SquareMatrix, decomposeUDUT) // NOLINT
{
  // clang-format off
  auto cov = conversions::SquareFromList<float32, 3, true>({
    {10, 2, 1},
    { 2, 5, 1},
    { 1, 1, 2}
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_TRUE(retVal.has_value());
  const auto [U, D] = retVal.value();

  // Verify U is unit upper triangular
  EXPECT_TRUE(U.isUnitUpperTriangular());

  // Verify reconstruction P = U * D * U^T
  const auto recomposed = (U * D) * U.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrix, decomposeUDUT_NotSymmetric_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = conversions::SquareFromList<float32, 2, true>({
    {10, 2},
    { 1, 5}
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrix, decomposeLDLT) // NOLINT
{
  // clang-format off
  auto cov = conversions::SquareFromList<float32, 6, true>({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_TRUE(retVal.has_value());
  const auto [L, D]     = retVal.value();
  const auto recomposed = (L * D) * L.transpose();
  for (auto row = 0; row < 6; row++)
  {
    for (auto col = 0; col < 6; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrix, decomposeLDLT_NotSymmetric_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = conversions::SquareFromList<float32, 2, true>({
    {10, -4},
    {-3, 13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrix, decomposeLDLT_SymmetricNotPositiveDefinite_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = conversions::SquareFromList<float32, 2, true>({
    {10, -3},
    {-3, -13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_FALSE(retVal.has_value());
}

// ============================================================================
// Matrix Property Check Tests
// ============================================================================

TEST(SquareMatrix, isOrthogonal_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.isOrthogonal();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isOrthogonal_NonOrthogonalMatrix__Fail) // NOLINT
{
  // Create a non-orthogonal matrix
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 2, true>({
    {1, 2},
    {3, 4}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isOrthogonal();

  EXPECT_FALSE(result);
}

TEST(SquareMatrix, isOrthogonal_OrthogonalMatrix__Success) // NOLINT
{
  // Create an orthogonal matrix (rotation matrix)
  const float32 cos_theta = std::cos(0.5F);
  const float32 sin_theta = std::sin(0.5F);

  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 2, true>({
    {cos_theta, -sin_theta},
    {sin_theta, cos_theta}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isOrthogonal(1e-5F);

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isUpperTriangular_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.isUpperTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isUpperTriangular_UpperTriangularMatrix__Success) // NOLINT
{
  // Create an upper triangular matrix
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    {1, 2, 3},
    {0, 4, 5},
    {0, 0, 6}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isUpperTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isUpperTriangular_NonUpperTriangularMatrix__Fail) // NOLINT
{
  // Create a non-upper triangular matrix
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    {1, 2, 3},
    {4, 5, 6},  // Has non-zero element below diagonal
    {7, 8, 9}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isUpperTriangular();

  EXPECT_FALSE(result);
}

TEST(SquareMatrix, isLowerTriangular_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.isLowerTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isLowerTriangular_LowerTriangularMatrix__Success) // NOLINT
{
  // Create a lower triangular matrix
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    {1, 0, 0},
    {2, 3, 0},
    {4, 5, 6}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isLowerTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isLowerTriangular_NonLowerTriangularMatrix__Fail) // NOLINT
{
  // Create a non-lower triangular matrix
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    {1, 2, 3},  // Has non-zero elements above diagonal
    {4, 5, 6},
    {7, 8, 9}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isLowerTriangular();

  EXPECT_FALSE(result);
}

TEST(SquareMatrix, hasUnitDiagonal_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.hasUnitDiagonal();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, hasUnitDiagonal_UnitDiagonalMatrix__Success) // NOLINT
{
  // Create a matrix with unit diagonal
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    {1, 2, 3},
    {4, 1, 6},
    {7, 8, 1}
  });
  // clang-format on

  // call UUT
  const auto result = mat.hasUnitDiagonal();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, hasUnitDiagonal_NonUnitDiagonalMatrix__Fail) // NOLINT
{
  // Create a matrix without unit diagonal
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    {2, 2, 3},  // Diagonal element is 2, not 1
    {4, 1, 6},
    {7, 8, 1}
  });
  // clang-format on

  // call UUT
  const auto result = mat.hasUnitDiagonal();

  EXPECT_FALSE(result);
}

TEST(SquareMatrix, isOrthogonal_WithTolerance__Success) // NOLINT
{
  // Create a matrix that's almost orthogonal but not exactly
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 2, true>({
    {0.999F, -0.01F},
    {0.01F, 0.999F}
  });
  // clang-format on

  // Should pass with loose tolerance
  EXPECT_TRUE(mat.isOrthogonal(0.02F));

  // Should fail with strict tolerance
  EXPECT_FALSE(mat.isOrthogonal(0.001F));
}

TEST(SquareMatrix, isUpperTriangular_WithTolerance__Success) // NOLINT
{
  // Create a matrix that's almost upper triangular
  // clang-format off
  const auto mat = conversions::SquareFromList<float32, 3, true>({
    {1, 2, 3},
    {0.001F, 4, 5},  // Small non-zero element below diagonal
    {0, 0, 6}
  });
  // clang-format on

  // Should pass with loose tolerance
  EXPECT_TRUE(mat.isUpperTriangular(0.01F));

  // Should fail with strict tolerance
  EXPECT_FALSE(mat.isUpperTriangular(0.0001F));
}

TEST(SquareMatrix, isOrthogonal_Double__Success) // NOLINT
{
  const auto identity = SquareMatrix<float64, 3, true>::Identity();

  // call UUT
  const auto result = identity.isOrthogonal();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isUpperTriangular_Double__Success) // NOLINT
{
  // Create an upper triangular matrix with double precision
  // clang-format off
  const auto mat = conversions::SquareFromList<float64, 3, true>({
    {1.0, 2.0, 3.0},
    {0.0, 4.0, 5.0},
    {0.0, 0.0, 6.0}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isUpperTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, hasUnitDiagonal_Double__Success) // NOLINT
{
  const auto identity = SquareMatrix<float64, 3, true>::Identity();

  // call UUT
  const auto result = identity.hasUnitDiagonal();

  EXPECT_TRUE(result);
}