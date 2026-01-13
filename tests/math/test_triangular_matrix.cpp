#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp"
#include "trackingLib/math/linalg/triangular_matrix.hpp" // IWYU pragma: keep

using namespace tracking::math;

TEST(TriangularMatrix, ctor_default) // NOLINT
{
  // clang-format off
  using TriangularMatrix = TriangularMatrix<float32, 3, false, true>;
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
  });
  // clang-format on

  // call UUT
  const TriangularMatrix triuMat{};

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, ctor_triu) // NOLINT
{
  // clang-format off
  using TriangularMatrix = TriangularMatrix<float32, 3, false, true>;
  const auto mat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3},
    {0, 5, 6},
    {0, 0, 9}
  });
  // clang-format on

  // call UUT
  const TriangularMatrix triuMat{mat};

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = row; col < 3; ++col)
    {
      EXPECT_FLOAT_EQ(expMat.at_unsafe(row, col), triuMat.at_unsafe(row, col));
    }
  }
}

// ============================================================================
// Matrix FromList ctor tests
// ============================================================================

/// \brief Helper class to support typed tests which wraps the IsRowMajor param into a type
template <bool IsRowMajor_>
struct MatrixStorageType
{
  static constexpr auto IsRowMajor = IsRowMajor_;
};

/// \brief Generic Matrix test class templatized by MatrixStorageType
template <typename MatrixStorageType>
class GTestTriangularMatrix: public ::testing::Test
{
};

using ::testing::Types;
// The list of types we want to test.
using MatrixStorageImplementations = Types<MatrixStorageType<true>, MatrixStorageType<false>>;
TYPED_TEST_SUITE(GTestTriangularMatrix, MatrixStorageImplementations);

TYPED_TEST(GTestTriangularMatrix, ctor_FromList_Upper__Success) // NOLINT
{
  // clang-format off
  // call UUT
  const auto result = TriangularMatrix<sint32, 3, false, TypeParam::IsRowMajor>::FromList({
      {1, 2, 3},
      {0, 5, 6},
      {0, 0, 9},
  });
  // clang-format on

  EXPECT_EQ(result.at_unsafe(0, 0), 1);
  EXPECT_EQ(result.at_unsafe(0, 1), 2);
  EXPECT_EQ(result.at_unsafe(0, 2), 3);
  EXPECT_EQ(result.at_unsafe(1, 1), 5);
  EXPECT_EQ(result.at_unsafe(1, 2), 6);
  EXPECT_EQ(result.at_unsafe(2, 2), 9);
}

TYPED_TEST(GTestTriangularMatrix, ctor_FromList_Lower__Success) // NOLINT
{
  // clang-format off
  // call UUT
  const auto result = TriangularMatrix<sint32, 3, true, TypeParam::IsRowMajor>::FromList({
      {1, 0, 0},
      {4, 5, 0},
      {7, 8, 9},
  });
  // clang-format on

  // Lower triangular: elements on and below diagonal should be preserved
  EXPECT_EQ(result.at_unsafe(0, 0), 1);
  EXPECT_EQ(result.at_unsafe(1, 0), 4);
  EXPECT_EQ(result.at_unsafe(1, 1), 5);
  EXPECT_EQ(result.at_unsafe(2, 0), 7);
  EXPECT_EQ(result.at_unsafe(2, 1), 8);
  EXPECT_EQ(result.at_unsafe(2, 2), 9);
}

TEST(TriangularMatrix, Identity) // NOLINT
{
  // clang-format off
  using TriangularMatrix = TriangularMatrix<float32, 3, false, true>;
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  auto triuMat{TriangularMatrix::Identity()};

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, setIdentity) // NOLINT
{
  // clang-format off
  using TriangularMatrix = TriangularMatrix<float32, 3, false, true>;
  TriangularMatrix triuMat{};
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  triuMat.setIdentity();

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, setBlock_lowerTopLeft) // NOLINT
{
  // clang-format off
  auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {1, 1, 0},
    {1, 1, 1}
  });
  const auto trilBlockMat = conversions::TriangularFromList<float32, 2, true, true>({
    {2, 0},
    {4, 3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {2, 0, 0},
    {4, 3, 0},
    {1, 1, 1}
  });
  // clang-format on

  // call UUT
  trilMat.setBlock<2, 2, 0, 0, 0, 0>(trilBlockMat);

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, setBlock_lowerBottomLeft) // NOLINT
{
  // clang-format off
  auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {1, 1, 0},
    {1, 1, 1}
  });
  const auto trilBlockMat = conversions::TriangularFromList<float32, 2, true, true>({
    {2, 0},
    {4, 3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {2, 1, 0},
    {4, 3, 1}
  });
  // clang-format on

  // call UUT
  trilMat.setBlock<2, 2, 0, 0, 1, 0>(trilBlockMat);

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, setBlock_lowerBottomRight) // NOLINT
{
  // clang-format off
  auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {1, 1, 0},
    {1, 1, 1}
  });
  const auto trilBlockMat = conversions::TriangularFromList<float32, 2, true, true>({
    {2, 0},
    {4, 3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {1, 2, 0},
    {1, 4, 3}
  });
  // clang-format on

  // call UUT
  trilMat.setBlock<2, 2, 0, 0, 1, 1>(trilBlockMat);

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, setBlock_upperTopLeft) // NOLINT
{
  // clang-format off
  auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 1, 1},
    {0, 1, 1},
    {0, 0, 1}
  });
  const auto triuBlockMat = conversions::TriangularFromList<float32, 2, false, true>({
    {2, 4},
    {0, 3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {2, 4, 1},
    {0, 3, 1},
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  triuMat.setBlock<2, 2, 0, 0, 0, 0>(triuBlockMat);

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, setBlock_upperTopRight) // NOLINT
{
  // clang-format off
  auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 1, 1},
    {0, 1, 1},
    {0, 0, 1}
  });
  const auto triuBlockMat = conversions::TriangularFromList<float32, 2, false, true>({
    {2, 4},
    {0, 3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 4},
    {0, 1, 3},
    {0, 0, 1}
  });
  // clang-format on
  // call UUT
  triuMat.setBlock<2, 2, 0, 0, 0, 1>(triuBlockMat);

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, setBlock_upperBottomRight) // NOLINT
{
  // clang-format off
  auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 1, 1},
    {0, 1, 1},
    {0, 0, 1}
  });
  const auto triuBlockMat = conversions::TriangularFromList<float32, 2, false, true>({
    {2, 4},
    {0, 3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 1, 1},
    {0, 2, 4},
    {0, 0, 3}
  });
  // clang-format on

  // call UUT
  triuMat.setBlock<2, 2, 0, 0, 1, 1>(triuBlockMat);

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_mat_lower) // NOLINT
{
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
  {1,  0,  0},
  {4,  5,  0},
  {6,  7,  8}
  });
  const auto mat = conversions::MatrixFromList<float32, 3, 4, true>({
    {1,  2,  3,  4},
    {5,  6,  7,  8},
    {9, 10, 11, 12}
  });
  const auto expMat = conversions::MatrixFromList<float32, 3, 4, true>({
    { 1,  2,  3,  4},
    {29, 38, 47, 56},
    {113, 134, 155, 176}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat * mat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_mat_upper) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  4,  6},
    {0,  5,  7},
    {0,  0,  8}
  });
  const auto mat = conversions::MatrixFromList<float32, 3, 4, true>({
    {1,  2,  3,  4},
    {5,  6,  7,  8},
    {9, 10, 11, 12}
  });
  const auto expMat = conversions::MatrixFromList<float32, 3, 4, true>({
    {75,  86,  97, 108},
    {88, 100, 112, 124},
    {72,  80,  88,  96}
  });
  // clang-format on

  // call UUT
  auto resMat = triuMat * mat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_lower_both) // NOLINT
{
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1,  0,  0},
    {4,  5,  0},
    {6,  7,  8}
  });
  const auto trilMat2 = conversions::TriangularFromList<float32, 3, true, true>({
    {8,  0,  0},
    {7,  5,  0},
    {6,  4,  1}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {  8,  0, 0},
    { 67, 25, 0},
    {145, 67, 8}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat * trilMat2;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_upper_both) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  4,  6},
    {0,  5,  7},
    {0,  0,  8}
  });
  const auto triuMat2 = conversions::TriangularFromList<float32, 3, false, true>({
    {8,  7,  6},
    {0,  5,  4},
    {0,  0,  1}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    { 8, 27, 28},
    { 0, 25, 27},
    { 0,  0,  8}
  });
  // clang-format on

  // call UUT
  auto resMat = triuMat * triuMat2;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_tria_opposite) // NOLINT
{
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1,  0,  0},
    {4,  5,  0},
    {6,  7,  8}
  });
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {8,  7,  6},
    {0,  5,  4},
    {0,  0,  1}
  });
  const auto expMat = conversions::SquareFromList<float32, 3, true>({
    { 8,  7,  6},
    {32, 53, 44},
    {48, 77, 72}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat * triuMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_diag_lower) // NOLINT
{
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1,  0,  0},
    {4,  5,  0},
    {6,  7,  8}
  });
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1,  0,  0},
    {4, 10,  0},
    {6, 14, 24}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat * diagMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_diag_upper) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  2,  3},
    {0,  5,  6},
    {0,  0,  8}
  });
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  4,  9},
    {0, 10, 18},
    {0,  0, 24}
  });
  // clang-format on

  // call UUT
  auto resMat = triuMat * diagMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_scal) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  2,  3},
    {0,  5,  6},
    {0,  0,  8}
  });
  const float32 scalar = 3;
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {3,  6,  9},
    {0, 15, 18},
    {0,  0, 24}
  });
  // clang-format on

  // call UUT
  auto resMat = triuMat * scalar;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_scal_upper_inplace) // NOLINT
{
  // clang-format off
  auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  2,  3},
    {0,  4,  5},
    {0,  0,  6}
  });
  const float32 scalar = 3;
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {3,  6,  9},
    {0, 12, 15},
    {0,  0, 18}
  });
  // clang-format on

  // call UUT
  triuMat *= scalar;

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_scal_lower_inplace) // NOLINT
{
  // clang-format off
  auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1,  0,  0},
    {2,  3,  0},
    {4,  5,  6}
  });
  const float32 scalar = 3;
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {3,  0,  0},
    {6,  9,  0},
    {12, 15, 18}
  });
  // clang-format on

  // call UUT
  trilMat *= scalar;

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, inverse_lower) // NOLINT
{
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1,  0,  0},
    {2,  4,  0},
    {3,  5,  6}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    { 1.000000000000000,                  0,                 0},
    {-0.500000000000000,  0.250000000000000,                 0},
    {-0.083333333333333, -0.208333333333333, 0.166666666666667}
  });
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  EXPECT_EQ(expMat._data, invMat._data);
}

TEST(TriangularMatrix, inverse_upper) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  2,  3},
    {0,  4,  5},
    {0,  0,  6}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1.000000000000000, -0.500000000000000, -0.083333333333333},
    {0,                 0.250000000000000, -0.208333333333333},
    {0,                 0,                 0.166666666666667}
  });
  // clang-format on

  // call UUT
  auto invMat = triuMat.inverse();

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = row; col < 3; ++col)
    {
      EXPECT_FLOAT_EQ(expMat.at_unsafe(row, col), invMat.at_unsafe(row, col));
    }
  }
}

TEST(TriangularMatrix, transpose_upper) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  2,  3},
    {0,  4,  5},
    {0,  0,  6}
  });
  using LowerTriangularMatrix3 = TriangularMatrix<float32, 3, true, true>;
  const LowerTriangularMatrix3 expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {2, 4, 0},
    {3, 5, 6}
  });
  // clang-format on

  // call UUT
  auto trilMat = triuMat.transpose();

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = 0; col <= row; ++col)
    {
      EXPECT_FLOAT_EQ(expMat.at_unsafe(row, col), trilMat.at_unsafe(row, col));
    }
  }
}

TEST(TriangularMatrix, solve_lower) // NOLINT
{
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1,  0,  0},
    {4,  5,  0},
    {6,  7,  8}
  });
  const auto bMat = conversions::MatrixFromList<float32, 3, 4, true>({
    {1,  2,  3,  4},
    {5,  6,  7,  8},
    {9, 10, 11, 12}
  });
  const auto expMat = conversions::MatrixFromList<float32, 3, 4, true>({
    {1.0,  2.0,  3.0,  4.0},
    {0.2, -0.4, -1.0, -1.6},
    {0.2,  0.1,  0.0, -0.1}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = row; col < 4; ++col)
    {
      EXPECT_FLOAT_EQ(expMat.at_unsafe(row, col), resMat.at_unsafe(row, col));
    }
  }
}

TEST(TriangularMatrix, solve_upper) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1,  4,  6},
    {0,  5,  7},
    {0,  0,  8}
  });
  const auto bMat = conversions::MatrixFromList<float32, 3, 4, true>({
    {1,  2,  3,  4},
    {5,  6,  7,  8},
    {9, 10, 11, 12}
  });
  const auto expMat = conversions::MatrixFromList<float32, 3, 4, true>({
    {-3.450, -3.30, -3.150, -3.00},
    {-0.575, -0.55, -0.525, -0.50},
    { 1.125,  1.25,  1.375,  1.50}
  });
  // clang-format on

  // call UUT
  auto resMat = triuMat.solve(bMat);

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = row; col < 4; ++col)
    {
      EXPECT_FLOAT_EQ(expMat.at_unsafe(row, col), resMat.at_unsafe(row, col));
    }
  }
}

TEST(TriangularMatrix, isUnitUpperTriangular_false) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3},
    {0, 1.01, 5},
    {0, 0, 0.9999999}
  });
  // clang-format on

  // call UUT
  auto result = triuMat.isUnitUpperTriangular();

  EXPECT_FALSE(result);
}

TEST(TriangularMatrix, isUnitUpperTriangular_true) // NOLINT
{
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3},
    {0, 1, 5},
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  auto result = triuMat.isUnitUpperTriangular();

  EXPECT_TRUE(result);
}

// ============================================================================
// Determinant Tests
// ============================================================================

TEST(TriangularMatrix, determinant_UpperTriangular_2x2__Success) // NOLINT
{
  // Create a 2x2 upper triangular matrix
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 2, false, true>({
    {1, 2},
    {0, 3}
  });
  // clang-format on

  // call UUT
  const auto result = triuMat.determinant();

  // Expected: 1 * 3 = 3 (product of diagonal elements)
  EXPECT_FLOAT_EQ(result, 3.0F);
}

TEST(TriangularMatrix, determinant_LowerTriangular_2x2__Success) // NOLINT
{
  // Create a 2x2 lower triangular matrix
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {1, 0},
    {2, 3}
  });
  // clang-format on

  // call UUT
  const auto result = trilMat.determinant();

  // Expected: 1 * 3 = 3 (product of diagonal elements)
  EXPECT_FLOAT_EQ(result, 3.0F);
}

TEST(TriangularMatrix, determinant_UpperTriangular_3x3__Success) // NOLINT
{
  // Create a 3x3 upper triangular matrix
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3},
    {0, 4, 5},
    {0, 0, 6}
  });
  // clang-format on

  // call UUT
  const auto result = triuMat.determinant();

  // Expected: 1 * 4 * 6 = 24 (product of diagonal elements)
  EXPECT_FLOAT_EQ(result, 24.0F);
}

TEST(TriangularMatrix, determinant_LowerTriangular_3x3__Success) // NOLINT
{
  // Create a 3x3 lower triangular matrix
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {2, 3, 0},
    {4, 5, 6}
  });
  // clang-format on

  // call UUT
  const auto result = trilMat.determinant();

  // Expected: 1 * 3 * 6 = 18 (product of diagonal elements)
  EXPECT_FLOAT_EQ(result, 18.0F);
}

TEST(TriangularMatrix, determinant_UnitTriangular_Upper__Success) // NOLINT
{
  // Create a 3x3 unit upper triangular matrix
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3},
    {0, 1, 4},
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  const auto result = triuMat.determinant();

  // Expected: 1 * 1 * 1 = 1 (unit triangular matrices have determinant 1)
  EXPECT_FLOAT_EQ(result, 1.0F);
}

TEST(TriangularMatrix, determinant_UnitTriangular_Lower__Success) // NOLINT
{
  // Create a 3x3 unit lower triangular matrix
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {2, 1, 0},
    {3, 4, 1}
  });
  // clang-format on

  // call UUT
  const auto result = trilMat.determinant();

  // Expected: 1 * 1 * 1 = 1 (unit triangular matrices have determinant 1)
  EXPECT_FLOAT_EQ(result, 1.0F);
}

TEST(TriangularMatrix, determinant_Singular_Upper__Success) // NOLINT
{
  // Create a singular upper triangular matrix (zero on diagonal)
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3},
    {0, 0, 4},
    {0, 0, 5}
  });
  // clang-format on

  // call UUT
  const auto result = triuMat.determinant();

  // Expected: 1 * 0 * 5 = 0 (singular matrix)
  EXPECT_FLOAT_EQ(result, 0.0F);
}

TEST(TriangularMatrix, determinant_Singular_Lower__Success) // NOLINT
{
  // Create a singular lower triangular matrix (zero on diagonal)
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {0, 0, 0},
    {1, 2, 0},
    {3, 4, 5}
  });
  // clang-format on

  // call UUT
  const auto result = trilMat.determinant();

  // Expected: 0 * 2 * 5 = 0 (singular matrix)
  EXPECT_FLOAT_EQ(result, 0.0F);
}

TEST(TriangularMatrix, determinant_Double_Upper__Success) // NOLINT
{
  // Create a 3x3 upper triangular matrix with double precision
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float64, 3, false, true>({
    {1.0, 2.0, 3.0},
    {0.0, 4.0, 5.0},
    {0.0, 0.0, 6.0}
  });
  // clang-format on

  // call UUT
  const auto result = triuMat.determinant();

  // Expected: 1.0 * 4.0 * 6.0 = 24.0
  EXPECT_DOUBLE_EQ(result, 24.0);
}

TEST(TriangularMatrix, determinant_Double_Lower__Success) // NOLINT
{
  // Create a 3x3 lower triangular matrix with double precision
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float64, 3, true, true>({
    {1.0, 0.0, 0.0},
    {2.0, 3.0, 0.0},
    {4.0, 5.0, 6.0}
  });
  // clang-format on

  // call UUT
  const auto result = trilMat.determinant();

  // Expected: 1.0 * 3.0 * 6.0 = 18.0
  EXPECT_DOUBLE_EQ(result, 18.0);
}

// ============================================================================
// Additional Triangular Solve and Inversion Tests
// ============================================================================

TEST(TriangularMatrix, solve_ForwardSubstitution_2x2__Success) // NOLINT
{
  // Test forward substitution for lower triangular matrix (2x2)
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {2, 0},
    {1, 3}
  });
  const auto bMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {4},
    {7}
  });
  const auto expMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {2.0},
    {1.6666667}
  });
  // clang-format on

  // call UUT - forward substitution for lower triangular
  auto resMat = trilMat.solve(bMat);

  EXPECT_NEAR(expMat.at_unsafe(0, 0), resMat.at_unsafe(0, 0), 1e-6);
  EXPECT_NEAR(expMat.at_unsafe(1, 0), resMat.at_unsafe(1, 0), 1e-6);
}

TEST(TriangularMatrix, solve_BackwardSubstitution_2x2__Success) // NOLINT
{
  // Test backward substitution for upper triangular matrix (2x2)
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 2, false, true>({
    {2, 1},
    {0, 3}
  });
  const auto bMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {5},
    {9}
  });
  const auto expMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {1.0},
    {3.0}
  });
  // clang-format on

  // call UUT - backward substitution for upper triangular
  auto resMat = triuMat.solve(bMat);

  EXPECT_NEAR(expMat.at_unsafe(0, 0), resMat.at_unsafe(0, 0), 1e-6);
  EXPECT_NEAR(expMat.at_unsafe(1, 0), resMat.at_unsafe(1, 0), 1e-6);
}

TEST(TriangularMatrix, solve_MultipleRHS_3x3__Success) // NOLINT
{
  // Test solving with multiple right-hand sides (3x3 system, 2 RHS)
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {2, 3, 0},
    {4, 5, 6}
  });
  const auto bMat = conversions::MatrixFromList<float32, 3, 2, true>({
    {1, 2},
    {5, 6},
    {9, 10}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  // Verify the solution by reconstructing: trilMat * resMat should ≈ bMat
  auto reconstructed = trilMat * resMat;

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = 0; col < 2; ++col)
    {
      EXPECT_NEAR(bMat.at_unsafe(row, col), reconstructed.at_unsafe(row, col), 1e-5);
    }
  }
}

TEST(TriangularMatrix, inverse_UnitTriangular_Upper__Success) // NOLINT
{
  // Test inversion of unit upper triangular matrix
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3},
    {0, 1, 4},
    {0, 0, 1}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, -2, 5},
    {0, 1, -4},
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  auto invMat = triuMat.inverse();

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = row; col < 3; ++col)
    {
      EXPECT_NEAR(expMat.at_unsafe(row, col), invMat.at_unsafe(row, col), 1e-6);
    }
  }
}

TEST(TriangularMatrix, inverse_UnitTriangular_Lower__Success) // NOLINT
{
  // Test inversion of unit lower triangular matrix
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {2, 1, 0},
    {3, 4, 1}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {-2, 1, 0},
    {5, -4, 1}
  });
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = 0; col <= row; ++col)
    {
      EXPECT_NEAR(expMat.at_unsafe(row, col), invMat.at_unsafe(row, col), 1e-6);
    }
  }
}

TEST(TriangularMatrix, solve_NearSingular_Lower__Success) // NOLINT
{
  // Test solving with near-singular lower triangular matrix
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {1, 0},
    {1, 0.0001}  // Very small diagonal element
  });
  const auto bMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {1},
    {1}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  // Should still work but with potential numerical issues
  EXPECT_NEAR(1.0, resMat.at_unsafe(0, 0), 1e-3);   // Less strict tolerance
  EXPECT_LT(std::abs(resMat.at_unsafe(1, 0)), 1e3); // Should not be infinite
}

TEST(TriangularMatrix, solve_NearSingular_Upper__Success) // NOLINT
{
  // Test solving with near-singular upper triangular matrix
  // clang-format off
  const auto triuMat = conversions::TriangularFromList<float32, 2, false, true>({
    {0.0001, 1},  // Very small diagonal element
    {0, 2}
  });
  const auto bMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {1},
    {2}
  });
  // clang-format on

  // call UUT
  auto resMat = triuMat.solve(bMat);

  // Should still work but with potential numerical issues
  EXPECT_NEAR(1.0, resMat.at_unsafe(1, 0), 1e-3);   // Less strict tolerance
  EXPECT_LT(std::abs(resMat.at_unsafe(0, 0)), 1e4); // Should not be extremely large
}

TEST(TriangularMatrix, inverse_NearSingular_Lower__Success) // NOLINT
{
  // Test inversion of near-singular lower triangular matrix
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {1, 0},
    {1, 0.001}  // Very small diagonal element
  });
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  // Should work but may have large values
  EXPECT_NEAR(1.0, invMat.at_unsafe(0, 0), 1e-3);
  EXPECT_NEAR(-1000.0, invMat.at_unsafe(1, 0), 1e-1); // Should be approximately -1/0.001
  EXPECT_GT(invMat.at_unsafe(1, 1), 1e2);             // Very large inverse element (1/0.001 = 1000)
}

TEST(TriangularMatrix, solve_IdentityMatrix__Success) // NOLINT
{
  // Test solving with identity matrix (should return the same as input)
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  const auto bMat = conversions::MatrixFromList<float32, 3, 2, true>({
    {1, 4},
    {2, 5},
    {3, 6}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  // Should be identical to input
  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = 0; col < 2; ++col)
    {
      EXPECT_FLOAT_EQ(bMat.at_unsafe(row, col), resMat.at_unsafe(row, col));
    }
  }
}

TEST(TriangularMatrix, inverse_IdentityMatrix__Success) // NOLINT
{
  // Test inversion of identity matrix (should return identity)
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  // Should be identity matrix
  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = 0; col <= row; ++col)
    {
      if (row == col)
      {
        EXPECT_FLOAT_EQ(1.0, invMat.at_unsafe(row, col));
      }
      else
      {
        EXPECT_FLOAT_EQ(0.0, invMat.at_unsafe(row, col));
      }
    }
  }
}

TEST(TriangularMatrix, solve_DiagonalMatrix__Success) // NOLINT
{
  // Test solving with diagonal matrix (special case of triangular)
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {2, 0, 0},
    {0, 3, 0},
    {0, 0, 4}
  });
  const auto bMat = conversions::MatrixFromList<float32, 3, 1, true>({
    {2},
    {6},
    {12}
  });
  const auto expMat = conversions::MatrixFromList<float32, 3, 1, true>({
    {1.0},
    {2.0},
    {3.0}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  for (auto row = 0; row < 3; ++row)
  {
    EXPECT_NEAR(expMat.at_unsafe(row, 0), resMat.at_unsafe(row, 0), 1e-6);
  }
}

TEST(TriangularMatrix, inverse_DiagonalMatrix__Success) // NOLINT
{
  // Test inversion of diagonal matrix (special case of triangular)
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {2, 0, 0},
    {0, 3, 0},
    {0, 0, 4}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
    {0.5, 0, 0},
    {0, 1.0/3.0, 0},
    {0, 0, 0.25}
  });
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = 0; col <= row; ++col)
    {
      EXPECT_NEAR(expMat.at_unsafe(row, col), invMat.at_unsafe(row, col), 1e-6);
    }
  }
}

TEST(TriangularMatrix, solve_LargeValues__Success) // NOLINT
{
  // Test solving with large values to check numerical stability
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {1000, 0},
    {2000, 3000}
  });
  const auto bMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {1000},
    {5000}
  });
  const auto expMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {1.0},
    {1.0}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  EXPECT_NEAR(expMat.at_unsafe(0, 0), resMat.at_unsafe(0, 0), 1e-3);
  EXPECT_NEAR(expMat.at_unsafe(1, 0), resMat.at_unsafe(1, 0), 1e-3);
}

TEST(TriangularMatrix, solve_SmallValues__Success) // NOLINT
{
  // Test solving with small values to check numerical stability
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {0.001, 0},
    {0.002, 0.003}
  });
  const auto bMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {0.001},
    {0.005}
  });
  const auto expMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {1.0},
    {1.0}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  EXPECT_NEAR(expMat.at_unsafe(0, 0), resMat.at_unsafe(0, 0), 1e-3);
  EXPECT_NEAR(expMat.at_unsafe(1, 0), resMat.at_unsafe(1, 0), 1e-3);
}

TEST(TriangularMatrix, solve_NegativeValues__Success) // NOLINT
{
  // Test solving with negative values
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {-1, 0},
    {2, -3}
  });
  const auto bMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {-1},
    {5}
  });
  const auto expMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {1.0},
    {-1.0}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  EXPECT_NEAR(expMat.at_unsafe(0, 0), resMat.at_unsafe(0, 0), 1e-6);
  EXPECT_NEAR(expMat.at_unsafe(1, 0), resMat.at_unsafe(1, 0), 1e-6);
}

TEST(TriangularMatrix, inverse_NegativeValues__Success) // NOLINT
{
  // Test inversion with negative values
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {-1, 0},
    {2, -3}
  });
  const auto expMat = conversions::TriangularFromList<float32, 2, true, true>({
    {-1, 0},
    {-2.0/3.0, -1.0/3.0}
  });
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  for (auto row = 0; row < 2; ++row)
  {
    for (auto col = 0; col <= row; ++col)
    {
      EXPECT_NEAR(expMat.at_unsafe(row, col), invMat.at_unsafe(row, col), 1e-6);
    }
  }
}

TEST(TriangularMatrix, solve_MixedPrecision__Success) // NOLINT
{
  // Test solving with double precision for better accuracy
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float64, 3, true, true>({
    {1.0, 0.0, 0.0},
    {2.0, 3.0, 0.0},
    {4.0, 5.0, 6.0}
  });
  const auto bMat = conversions::MatrixFromList<float64, 3, 1, true>({
    {1.0},
    {5.0},
    {9.0}
  });
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  // Verify the solution by reconstructing: trilMat * resMat should ≈ bMat
  auto reconstructed = trilMat * resMat;

  for (auto row = 0; row < 3; ++row)
  {
    EXPECT_NEAR(bMat.at_unsafe(row, 0), reconstructed.at_unsafe(row, 0), 1e-14);
  }
}

TEST(TriangularMatrix, inverse_MixedPrecision__Success) // NOLINT
{
  // Test inversion with double precision for better accuracy
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float64, 2, true, true>({
    {2.0, 0.0},
    {1.0, 3.0}
  });
  const auto expMat = conversions::TriangularFromList<float64, 2, true, true>({
    {0.5, 0.0},
    {-1.0/6.0, 1.0/3.0}
  });
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  for (auto row = 0; row < 2; ++row)
  {
    for (auto col = 0; col <= row; ++col)
    {
      EXPECT_NEAR(expMat.at_unsafe(row, col), invMat.at_unsafe(row, col), 1e-14);
    }
  }
}

TEST(TriangularMatrix, solve_VerifySolution__Success) // NOLINT
{
  // Test that the solution actually satisfies the original equation
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {2, 0},
    {1, 3}
  });
  const auto bMat = conversions::MatrixFromList<float32, 2, 1, true>({
    {4},
    {7}
  });
  // clang-format on

  // call UUT
  auto xMat = trilMat.solve(bMat);

  // Verify that trilMat * xMat ≈ bMat
  auto reconstructed = trilMat * xMat;
  EXPECT_NEAR(bMat.at_unsafe(0, 0), reconstructed.at_unsafe(0, 0), 1e-5);
  EXPECT_NEAR(bMat.at_unsafe(1, 0), reconstructed.at_unsafe(1, 0), 1e-5);
}

TEST(TriangularMatrix, inverse_VerifyInverse__Success) // NOLINT
{
  // Test that the inverse actually satisfies T * T^(-1) = I
  // clang-format off
  const auto trilMat = conversions::TriangularFromList<float32, 2, true, true>({
    {2, 0},
    {1, 3}
  });
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  // Verify that trilMat * invMat ≈ I (only check the triangular part)
  auto product = trilMat * invMat;
  EXPECT_NEAR(1.0, product.at_unsafe(0, 0), 1e-5);
  EXPECT_NEAR(0.0, product.at_unsafe(1, 0), 1e-5);
  EXPECT_NEAR(1.0, product.at_unsafe(1, 1), 1e-5);
}
