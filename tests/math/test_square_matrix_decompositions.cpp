#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"
#include "trackingLib/math/linalg/diagonal_matrix.hpp"
#include "trackingLib/math/linalg/square_matrix.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/triangular_matrix.hpp"

using namespace tracking::math;

// Helper function to create test matrices
template <typename ValueType_, sint32 Size_>
auto createSymmetricPositiveDefiniteMatrix() -> SquareMatrix<ValueType_, Size_, true>
{
  // Create a symmetric positive definite matrix
  SquareMatrix<ValueType_, Size_, true> mat{};
  for (auto i = 0; i < Size_; ++i)
  {
    for (auto j = 0; j < Size_; ++j)
    {
      // Create a symmetric matrix with dominant diagonal
      if (i == j)
      {
        mat.at_unsafe(i, j) = static_cast<ValueType_>(Size_ + 1); // Diagonal dominance
      }
      else
      {
        mat.at_unsafe(i, j) = static_cast<ValueType_>(std::abs(i - j));
      }
    }
  }
  return mat;
}

// Helper function to create ill-conditioned matrix
template <typename ValueType_, sint32 Size_>
auto createIllConditionedMatrix() -> SquareMatrix<ValueType_, Size_, true>
{
  SquareMatrix<ValueType_, Size_, true> mat{};
  for (auto i = 0; i < Size_; ++i)
  {
    for (auto j = 0; j < Size_; ++j)
    {
      mat.at_unsafe(i, j) = static_cast<ValueType_>(1.0) / (static_cast<ValueType_>(i + j + 1));
    }
  }
  return mat;
}

// ============================================================================
// Householder QR Decomposition Tests
// ============================================================================

TEST(SquareMatrixDecompositions, householderQR__Success) // NOLINT
{
  // Create a test matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
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

TEST(SquareMatrixDecompositions, householderQR_OrthogonalityOfQ__Success) // NOLINT
{
  // Create a test matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  // clang-format on

  // call UUT
  const auto [Q, R] = mat.householderQR();

  // Check that Q is orthogonal (Q^T * Q = I)
  EXPECT_TRUE(Q.isOrthogonal());
}

TEST(SquareMatrixDecompositions, householderQR_UpperTriangularR__Success) // NOLINT
{
  // Create a test matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  // clang-format on

  // call UUT
  const auto [Q, R] = mat.householderQR();

  // Check that R is upper triangular
  EXPECT_TRUE(R.isUpperTriangular());
}

TEST(SquareMatrixDecompositions, householderQR_Reconstruction__Success) // NOLINT
{
  // Create a test matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  // clang-format on

  // call UUT
  const auto [Q, R] = mat.householderQR();

  // Check reconstruction: Q * R should equal original matrix
  const auto recomposed = Q * R;
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_NEAR(mat.at_unsafe(row, col), recomposed.at_unsafe(row, col), 1e-5);
    }
  }
}

// ============================================================================
// LLT Decomposition Tests
// ============================================================================

TEST(SquareMatrixDecompositions, decomposeLLT__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_TRUE(retVal.has_value());
  const auto& L = retVal.value();

  // Check reconstruction: L * L^T should equal original matrix
  const auto recomposed = L * L.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeLLT_Reconstruction__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_TRUE(retVal.has_value());
  const auto& L = retVal.value();

  // Check reconstruction: L * L^T should equal original matrix
  const auto recomposed = L * L.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_NEAR(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col), 1e-6);
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeLLT_NotSymmetric_ExpectError) // NOLINT
{
  // Create a non-symmetric matrix
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, -4},
    {-3, 13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrixDecompositions, decomposeLLT_SymmetricNotPositiveDefinite_ExpectError) // NOLINT
{
  // Create a symmetric but not positive definite matrix
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, -3},
    {-3, -13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_FALSE(retVal.has_value());
}

// ============================================================================
// LDLT Decomposition Tests
// ============================================================================

TEST(SquareMatrixDecompositions, decomposeLDLT__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_TRUE(retVal.has_value());
  const auto [L, D]     = retVal.value();
  const auto recomposed = (L * D) * L.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeLDLT_Reconstruction__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_TRUE(retVal.has_value());
  const auto [L, D]     = retVal.value();
  const auto recomposed = (L * D) * L.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_NEAR(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col), 1e-6);
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeLDLT_UnitDiagonalL__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_TRUE(retVal.has_value());
  const auto [L, D] = retVal.value();

  // Check that L has unit diagonal
  EXPECT_TRUE(L.hasUnitDiagonal());
}

TEST(SquareMatrixDecompositions, decomposeLDLT_NotSymmetric_ExpectError) // NOLINT
{
  // Create a non-symmetric matrix
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, -4},
    {-3, 13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrixDecompositions, decomposeLDLT_SymmetricNotPositiveDefinite_ExpectError) // NOLINT
{
  // Create a symmetric but not positive definite matrix
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, -3},
    {-3, -13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_FALSE(retVal.has_value());
}

// ============================================================================
// UDUT Decomposition Tests
// ============================================================================

TEST(SquareMatrixDecompositions, decomposeUDUT__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

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

TEST(SquareMatrixDecompositions, decomposeUDUT_Reconstruction__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_TRUE(retVal.has_value());
  const auto [U, D] = retVal.value();

  // Verify reconstruction P = U * D * U^T
  const auto recomposed = (U * D) * U.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_NEAR(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col), 1e-6);
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeUDUT_UnitDiagonalU__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_TRUE(retVal.has_value());
  const auto [U, D] = retVal.value();

  // Check that U has unit diagonal
  EXPECT_TRUE(U.hasUnitDiagonal());
}

TEST(SquareMatrixDecompositions, decomposeUDUT_NumericalStability__Success) // NOLINT
{
  // Create an ill-conditioned symmetric matrix
  auto cov = createIllConditionedMatrix<float32, 3>();
  cov.symmetrize(); // Ensure symmetry

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_TRUE(retVal.has_value());
  const auto [U, D] = retVal.value();

  // Verify reconstruction P = U * D * U^T
  const auto recomposed = (U * D) * U.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_NEAR(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col), 1e-4); // Looser tolerance for ill-conditioned
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeUDUT_NotSymmetric_ExpectError) // NOLINT
{
  // Create a non-symmetric matrix
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, 2},
    { 1, 5}
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_FALSE(retVal.has_value());
}

// ============================================================================
// Tests for Different Value Types
// ============================================================================

TEST(SquareMatrixDecompositions, householderQR_Double__Success) // NOLINT
{
  // Create a test matrix with double precision
  // clang-format off
  const auto mat = SquareMatrix<float64, 3, true>::FromList({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  // clang-format on

  // call UUT
  const auto [Q, R] = mat.householderQR();

  // Check reconstruction
  const auto recomposed = Q * R;
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_DOUBLE_EQ(mat.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeLLT_Double__Success) // NOLINT
{
  // Create a symmetric positive definite matrix with double precision
  auto cov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_TRUE(retVal.has_value());
  const auto& L = retVal.value();

  // Check reconstruction
  const auto recomposed = L * L.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_DOUBLE_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeLDLT_Double__Success) // NOLINT
{
  // Create a symmetric positive definite matrix with double precision
  auto cov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_TRUE(retVal.has_value());
  const auto [L, D]     = retVal.value();
  const auto recomposed = (L * D) * L.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_DOUBLE_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrixDecompositions, decomposeUDUT_Double__Success) // NOLINT
{
  // Create a symmetric positive definite matrix with double precision
  auto cov = createSymmetricPositiveDefiniteMatrix<float64, 3>();

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_TRUE(retVal.has_value());
  const auto [U, D] = retVal.value();

  // Verify reconstruction
  const auto recomposed = (U * D) * U.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_DOUBLE_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}