#include "gtest/gtest.h"
#include "trackingLib/math/linalg/triangular_matrix.hpp" // IWYU pragma: keep

TEST(TriangularMatrix, ctor_default) // NOLINT
{
  // clang-format off
  using TriangularMatrix = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto expMat = TriangularMatrix::FromList({
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
  using TriangularMatrix = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto mat = TriangularMatrix::FromList({
    {1, 2, 3}, 
    {4, 5, 6}, 
    {7, 8, 9}
  });
  const auto expMat = TriangularMatrix::FromList({
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

TEST(TriangularMatrix, Identity) // NOLINT
{
  // clang-format off
  using TriangularMatrix = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto expMat = TriangularMatrix::FromList({
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
  using TriangularMatrix = tracking::math::TriangularMatrix<float32, 3, false, true>;
  TriangularMatrix triuMat{}; 
  const auto expMat = TriangularMatrix::FromList({
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
  using LowerTriangularMatrix2 = tracking::math::TriangularMatrix<float32, 2, true, true>;
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  auto trilMat = LowerTriangularMatrix3::FromList({
    {1, 0, 0}, 
    {1, 1, 0}, 
    {1, 1, 1}
  });
  const auto trilBlockMat = LowerTriangularMatrix2::FromList({
    {2, 0}, 
    {4, 3}
  });
  const auto expMat = LowerTriangularMatrix3::FromList({
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
  using LowerTriangularMatrix2 = tracking::math::TriangularMatrix<float32, 2, true, true>;
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  auto trilMat = LowerTriangularMatrix3::FromList({
    {1, 0, 0}, 
    {1, 1, 0}, 
    {1, 1, 1}
  });
  const auto trilBlockMat = LowerTriangularMatrix2::FromList({
    {2, 0}, 
    {4, 3}
  });
  const auto expMat = LowerTriangularMatrix3::FromList({
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
  using LowerTriangularMatrix2 = tracking::math::TriangularMatrix<float32, 2, true, true>;
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  auto trilMat = LowerTriangularMatrix3::FromList({
    {1, 0, 0}, 
    {1, 1, 0}, 
    {1, 1, 1}
  });
  const auto trilBlockMat = LowerTriangularMatrix2::FromList({
    {2, 0}, 
    {4, 3}
  });
  const auto expMat = LowerTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix2 = tracking::math::TriangularMatrix<float32, 2, false, true>;
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  auto triuMat = UpperTriangularMatrix3::FromList({
    {1, 1, 1},
    {0, 1, 1},
    {0, 0, 1}
  });
  const auto triuBlockMat = UpperTriangularMatrix2::FromList({
    {2, 4},
    {0, 3}
  });
  const auto expMat = UpperTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix2 = tracking::math::TriangularMatrix<float32, 2, false, true>;
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  auto triuMat = UpperTriangularMatrix3::FromList({
    {1, 1, 1},
    {0, 1, 1},
    {0, 0, 1}
  });
  const auto triuBlockMat = UpperTriangularMatrix2::FromList({
    {2, 4},
    {0, 3}
  });
  const auto expMat = UpperTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix2 = tracking::math::TriangularMatrix<float32, 2, false, true>;
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  auto triuMat = UpperTriangularMatrix3::FromList({
    {1, 1, 1},
    {0, 1, 1},
    {0, 0, 1}
  });
  const auto triuBlockMat = UpperTriangularMatrix2::FromList({
    {2, 4},
    {0, 3}
  });
  const auto expMat = UpperTriangularMatrix3::FromList({
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
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  using Matrix34 = tracking::math::Matrix<float32, 3, 4, true>;
  const auto trilMat = LowerTriangularMatrix3::FromList({
  {1,  0,  0}, 
  {4,  5,  0}, 
  {6,  7,  8}
  });
  const auto mat = Matrix34::FromList({
    {1,  2,  3,  4}, 
    {5,  6,  7,  8}, 
    {9, 10, 11, 12}
  });
  const auto expMat = Matrix34::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  using Matrix34 = tracking::math::Matrix<float32, 3, 4, true>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {1,  4,  6}, 
    {0,  5,  7}, 
    {0,  0,  8}
  });
  const auto mat = Matrix34::FromList({
    {1,  2,  3,  4}, 
    {5,  6,  7,  8}, 
    {9, 10, 11, 12}
  });
  const auto expMat = Matrix34::FromList({
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
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  const auto trilMat = LowerTriangularMatrix3::FromList({
    {1,  0,  0}, 
    {4,  5,  0}, 
    {6,  7,  8}
  });
  const auto trilMat2 = LowerTriangularMatrix3::FromList({
    {8,  0,  0}, 
    {7,  5,  0}, 
    {6,  4,  1}
  });
  const auto expMat = LowerTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {1,  4,  6}, 
    {0,  5,  7}, 
    {0,  0,  8}
  });
  const auto triuMat2 = UpperTriangularMatrix3::FromList({
    {8,  7,  6}, 
    {0,  5,  4}, 
    {0,  0,  1}
  });
  const auto expMat = UpperTriangularMatrix3::FromList({
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
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto trilMat = LowerTriangularMatrix3::FromList({
    {1,  0,  0}, 
    {4,  5,  0}, 
    {6,  7,  8}
  });
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {8,  7,  6}, 
    {0,  5,  4}, 
    {0,  0,  1}
  });
  const auto expMat = tracking::math::SquareMatrix<float32, 3, true>::FromList({
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
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  using DiagonalMatrix3 = tracking::math::DiagonalMatrix<float32, 3>;
  const auto trilMat = LowerTriangularMatrix3::FromList({
    {1,  0,  0}, 
    {4,  5,  0}, 
    {6,  7,  8}
  });
  const auto diagMat = DiagonalMatrix3::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto expMat = LowerTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  using DiagonalMatrix3 = tracking::math::DiagonalMatrix<float32, 3>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {1,  2,  3}, 
    {0,  5,  6}, 
    {0,  0,  8}
  });
  const auto diagMat = DiagonalMatrix3::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto expMat = UpperTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {1,  2,  3}, 
    {0,  5,  6}, 
    {0,  0,  8}
  });
  const float32 scalar = 3;
  const auto expMat = UpperTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  auto triuMat = UpperTriangularMatrix3::FromList({
    {1,  2,  3}, 
    {0,  4,  5}, 
    {0,  0,  6}
  });
  const float32 scalar = 3;
  const auto expMat = UpperTriangularMatrix3::FromList({
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
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  auto trilMat = LowerTriangularMatrix3::FromList({
    {1,  0,  0}, 
    {2,  3,  0}, 
    {4,  5,  6}
  });
  const float32 scalar = 3;
  const auto expMat = LowerTriangularMatrix3::FromList({
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
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  const auto trilMat = LowerTriangularMatrix3::FromList({
    {1,  0,  0}, 
    {2,  4,  0}, 
    {3,  5,  6}
  });
  const auto expMat = LowerTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {1,  2,  3}, 
    {0,  4,  5}, 
    {0,  0,  6}
  });
  const auto expMat = UpperTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {1,  2,  3}, 
    {0,  4,  5}, 
    {0,  0,  6}
  });
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  const LowerTriangularMatrix3 expMat = LowerTriangularMatrix3::FromList({
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
  using LowerTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, true, true>;
  const auto trilMat = LowerTriangularMatrix3::FromList({
    {1,  0,  0}, 
    {4,  5,  0}, 
    {6,  7,  8}
  });
  const auto bMat = tracking::math::Matrix<float32, 3, 4, true>::FromList({
    {1,  2,  3,  4}, 
    {5,  6,  7,  8}, 
    {9, 10, 11, 12}
  });
  const auto expMat = tracking::math::Matrix<float32, 3, 4, true>::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {1,  4,  6}, 
    {0,  5,  7}, 
    {0,  0,  8}
  });
  const auto bMat = tracking::math::Matrix<float32, 3, 4, true>::FromList({
    {1,  2,  3,  4}, 
    {5,  6,  7,  8}, 
    {9, 10, 11, 12}
  });
  const auto expMat = tracking::math::Matrix<float32, 3, 4, true>::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
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
  using UpperTriangularMatrix3 = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto triuMat = UpperTriangularMatrix3::FromList({
    {1, 2, 3}, 
    {0, 1, 5}, 
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  auto result = triuMat.isUnitUpperTriangular();

  EXPECT_TRUE(result);
}
