#include "gtest/gtest.h"
#include "trackingLib/math/linalg/square_matrix.hpp" // IWYU pragma: keep

TEST(SquareMatrix, householderQR) // NOLINT
{
  // Create a square matrix for testing
  // clang-format off
  using FloatSquareMatType = tracking::math::SquareMatrix<float32, 3, true>;
  const auto mat = FloatSquareMatType::FromList({
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
  using FloatSquareMatType = tracking::math::SquareMatrix<float32, 3, true>;
  const auto mat = FloatSquareMatType::FromList({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  const auto expInvMat = FloatSquareMatType::FromList({
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
  auto cov = tracking::math::SquareMatrix<float32, 6, true>::FromList({
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
  auto cov = tracking::math::SquareMatrix<float32, 2, true>::FromList({
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
  auto cov = tracking::math::SquareMatrix<float32, 2, true>::FromList({
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
  auto mat = tracking::math::SquareMatrix<float32, 3, true>::FromList({
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

TEST(SquareMatrix, ctor_from_diag) // NOLINT
{
  using DiagMatType = tracking::math::DiagonalMatrix<float32, 3>;
  const auto diag   = DiagMatType::FromList({1, 2, 3});

  // call UUT
  const tracking::math::SquareMatrix<float32, 3, true> mat(diag);

  EXPECT_FLOAT_EQ(mat.at_unsafe(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(1, 1), 2.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(2, 2), 3.0F);

  // Off-diagonal should be zero
  EXPECT_FLOAT_EQ(mat.at_unsafe(0, 1), 0.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(1, 0), 0.0F);
}

TEST(SquareMatrix, decomposeUDUT) // NOLINT
{
  // clang-format off
  auto cov = tracking::math::SquareMatrix<float32, 3, true>::FromList({
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
  auto cov = tracking::math::SquareMatrix<float32, 2, true>::FromList({
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
  auto cov = tracking::math::SquareMatrix<float32, 6, true>::FromList({
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
  auto cov = tracking::math::SquareMatrix<float32, 2, true>::FromList({
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
  auto cov = tracking::math::SquareMatrix<float32, 2, true>::FromList({
    {10, -3},
    {-3, -13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_FALSE(retVal.has_value());
}