#include "gtest/gtest.h"
#include "math/linalg/matrix.h"
#include "trackingLib/math/linalg/square_matrix.hpp"
#include "trackingLib/math/linalg/triangular_matrix.hpp"
#include "trackingLib/math/linalg/diagonal_matrix.hpp"

TEST(TriangularMatrix, ctor_default) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{0, 0, 0}, 
     {0, 0, 0}, 
     {0, 0, 0}});
  // clang-format on

  // call UUT
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat{};

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, ctor_triu) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, false> mat(
    {{1, 2, 3}, 
     {4, 5, 6}, 
     {7, 8, 9}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 2, 3}, 
     {0, 5, 6}, 
     {0, 0, 9}});
  // clang-format on

  // call UUT
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat{mat};

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, ctor_square) // NOLINT
{
  // clang-format off
  const tracking::math::SquareMatrix<float32, 3> mat(
    {{1, 2, 3}, 
     {4, 5, 6}, 
     {7, 8, 9}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 2, 3}, 
     {0, 5, 6}, 
     {0, 0, 9}});
  // clang-format on

  // call UUT
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat{mat};

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, dtor) // NOLINT
{
  // clang-format off
  auto* mat = new tracking::math::TriangularMatrix<float32, 3, false>(
    {{1, 2, 3}, 
     {0, 5, 6}, 
     {0, 0, 9}});
  // clang-format on

  // call UUT
  delete mat;
}

TEST(TriangularMatrix, Identity) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 0, 0}, 
     {0, 1, 0}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  auto triuMat{tracking::math::TriangularMatrix<float32, 3, false>::Identity()};

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, setIdentity) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat{}; 
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 0, 0}, 
     {0, 1, 0}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  triuMat.setIdentity();

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, setBlock_lowerTopLeft) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1, 0, 0}, 
     {1, 1, 0}, 
     {1, 1, 1}});
  const tracking::math::TriangularMatrix<float32, 2, true> trilBlockMat(
    {{2, 0},  
     {4, 3}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{2, 0, 0}, 
     {4, 3, 0}, 
     {1, 1, 1}});
  // clang-format on

  // call UUT
  trilMat.setBlock<2, 2, 0, 0, 0, 0>(trilBlockMat);

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, setBlock_lowerBottomLeft) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1, 0, 0}, 
     {1, 1, 0}, 
     {1, 1, 1}});
  const tracking::math::TriangularMatrix<float32, 2, true> trilBlockMat(
    {{2, 0},  
     {4, 3}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{1, 0, 0}, 
     {2, 1, 0}, 
     {4, 3, 1}});
  // clang-format on

  // call UUT
  trilMat.setBlock<2, 2, 0, 0, 1, 0>(trilBlockMat);

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, setBlock_lowerBottomRight) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1, 0, 0}, 
     {1, 1, 0}, 
     {1, 1, 1}});
  const tracking::math::TriangularMatrix<float32, 2, true> trilBlockMat(
    {{2, 0},  
     {4, 3}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{1, 0, 0}, 
     {1, 2, 0}, 
     {1, 4, 3}});
  // clang-format on

  // call UUT
  trilMat.setBlock<2, 2, 0, 0, 1, 1>(trilBlockMat);

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, setBlock_upperTopLeft) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1, 1, 1}, 
     {0, 1, 1}, 
     {0, 0, 1}});
  const tracking::math::TriangularMatrix<float32, 2, false> triuBlockMat(
    {{2, 4},  
     {0, 3}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{2, 4, 1}, 
     {0, 3, 1}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  triuMat.setBlock<2, 2, 0, 0, 0, 0>(triuBlockMat);

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, setBlock_upperTopRight) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1, 1, 1}, 
     {0, 1, 1}, 
     {0, 0, 1}});
  const tracking::math::TriangularMatrix<float32, 2, false> triuBlockMat(
    {{2, 4},  
     {0, 3}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 2, 4}, 
     {0, 1, 3}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  triuMat.setBlock<2, 2, 0, 0, 0, 1>(triuBlockMat);

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, setBlock_upperBottomRight) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1, 1, 1}, 
     {0, 1, 1}, 
     {0, 0, 1}});
  const tracking::math::TriangularMatrix<float32, 2, false> triuBlockMat(
    {{2, 4},  
     {0, 3}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 1, 1}, 
     {0, 2, 4}, 
     {0, 0, 3}});
  // clang-format on

  // call UUT
  triuMat.setBlock<2, 2, 0, 0, 1, 1>(triuBlockMat);

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_mat_lower) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1,  0,  0}, 
     {4,  5,  0}, 
     {6,  7,  8}});
  const tracking::math::Matrix<float32, 3, 4> mat(
    {{1,  2,  3,  4}, 
     {5,  6,  7,  8}, 
     {9, 10, 11, 12}});
  const tracking::math::Matrix<float32, 3, 4> expMat(
    {{  1,   2,   3,   4}, 
     { 29,  38,  47,  56}, 
     {113, 134, 155, 176}});
  // clang-format on

  // call UUT
  auto resMat = trilMat * mat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_mat_upper) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1,  4,  6}, 
     {0,  5,  7}, 
     {0,  0,  8}});
  const tracking::math::Matrix<float32, 3, 4> mat(
    {{1,  2,  3,  4}, 
     {5,  6,  7,  8}, 
     {9, 10, 11, 12}});
  const tracking::math::Matrix<float32, 3, 4> expMat(
    {{75,  86,  97, 108}, 
     {88, 100, 112, 124}, 
     {72,  80,  88,  96}});
  // clang-format on

  // call UUT
  auto resMat = triuMat * mat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_lower_both) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1,  0,  0}, 
     {4,  5,  0}, 
     {6,  7,  8}});
  const tracking::math::TriangularMatrix<float32, 3, true> trilMat2(
    {{8,  0,  0}, 
     {7,  5,  0}, 
     {6,  4,  1}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{  8,  0, 0}, 
     { 67, 25, 0}, 
     {145, 67, 8}});
  // clang-format on

  // call UUT
  auto resMat = trilMat * trilMat2;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_upper_both) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1,  4,  6}, 
     {0,  5,  7}, 
     {0,  0,  8}});
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat2(
    {{8,  7,  6}, 
     {0,  5,  4}, 
     {0,  0,  1}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{8, 27, 28}, 
     {0, 25, 27}, 
     {0,  0,  8}});
  // clang-format on

  // call UUT
  auto resMat = triuMat * triuMat2;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_tria_opposite) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1,  0,  0}, 
     {4,  5,  0}, 
     {6,  7,  8}});
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{8,  7,  6}, 
     {0,  5,  4}, 
     {0,  0,  1}});
  const tracking::math::SquareMatrix<float32, 3> expMat(
    {{ 8,  7,  6}, 
     {32, 53, 44}, 
     {48, 77, 72}});
  // clang-format on

  // call UUT
  auto resMat = trilMat * triuMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_diag_lower) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1,  0,  0}, 
     {4,  5,  0}, 
     {6,  7,  8}});
  const tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const tracking::math::Matrix<float32, 3, 3> expMat(
    {{ 1,  0,  0}, 
     { 4, 10,  0}, 
     { 6, 14, 24}});
  // clang-format on

  // call UUT
  auto resMat = trilMat * diagMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_diag_upper) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1,  2,  3}, 
     {0,  5,  6}, 
     {0,  0,  8}});
  const tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const tracking::math::Matrix<float32, 3, 3> expMat(
    {{ 1,  4,  9}, 
     { 0, 10, 18}, 
     { 0,  0, 24}});
  // clang-format on

  // call UUT
  auto resMat = triuMat * diagMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_scal) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1,  2,  3}, 
     {0,  5,  6}, 
     {0,  0,  8}});
  const float32 scalar = 3;
  const tracking::math::Matrix<float32, 3, 3> expMat(
    {{ 3,  6,  9}, 
     { 0, 15, 18}, 
     { 0,  0, 24}});
  // clang-format on

  // call UUT
  auto resMat = triuMat * scalar;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_scal_upper_inplace) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1,  2,  3}, 
     {0,  4,  5}, 
     {0,  0,  6}});
  const float32 scalar = 3;
  const tracking::math::Matrix<float32, 3, 3> expMat(
    {{ 3,  6,  9}, 
     { 0, 12, 15}, 
     { 0,  0, 18}});
  // clang-format on

  // call UUT
  triuMat *= scalar;

  EXPECT_EQ(expMat._data, triuMat._data);
}

TEST(TriangularMatrix, op_mul_rhs_scal_lower_inplace) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1,  0,  0}, 
     {2,  3,  0}, 
     {4,  5,  6}});
  const float32 scalar = 3;
  const tracking::math::Matrix<float32, 3, 3> expMat(
    {{ 3,  0,  0}, 
     { 6,  9,  0}, 
     {12, 15, 18}});
  // clang-format on

  // call UUT
  trilMat *= scalar;

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, inverse_lower) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1, 0, 0}, 
     {2, 4, 0}, 
     {3, 5, 6}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{ 1.000000000000000,                  0,                 0},
     {-0.500000000000000,  0.250000000000000,                 0},
     {-0.083333333333333, -0.208333333333333, 0.166666666666667}});
  // clang-format on

  // call UUT
  auto invMat = trilMat.inverse();

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = 0; col <= row; ++col)
    {
      EXPECT_FLOAT_EQ(expMat(row, col), invMat(row, col));
    }
  }
}

TEST(TriangularMatrix, inverse_upper) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1, 2, 3}, 
     {0, 4, 5}, 
     {0, 0, 6}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, -0.5,  -0.083333333333333},
     {0,  0.25, -0.208333333333333},
     {0,  0,     0.166666666666667}});
  // clang-format on

  // call UUT
  auto invMat = triuMat.inverse();

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = row; col < 3; ++col)
    {
      EXPECT_FLOAT_EQ(expMat(row, col), invMat(row, col));
    }
  }
}

TEST(TriangularMatrix, transpose_upper) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1, 2, 3}, 
     {0, 4, 5}, 
     {0, 0, 6}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{1, 0, 0}, 
     {2, 4, 0}, 
     {3, 5, 6}});
  // clang-format on

  // call UUT
  auto trilMat = triuMat.transpose();

  EXPECT_EQ(expMat._data, trilMat._data);
}

TEST(TriangularMatrix, solve_lower) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, true> trilMat(
    {{1,  0,  0}, 
     {4,  5,  0}, 
     {6,  7,  8}});
  const tracking::math::Matrix<float32, 3, 4> bMat(
    {{1,  2,  3,  4}, 
     {5,  6,  7,  8}, 
     {9, 10, 11, 12}});
  const tracking::math::Matrix<float32, 3, 4> expMat(
    {{1.0,  2.0,  3.0,  4.0}, 
     {0.2, -0.4, -1.0, -1.6}, 
     {0.2,  0.1,  0.0, -0.1}});
  // clang-format on

  // call UUT
  auto resMat = trilMat.solve(bMat);

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = row; col < 4; ++col)
    {
      EXPECT_FLOAT_EQ(expMat(row, col), resMat(row, col));
    }
  }
}

TEST(TriangularMatrix, solve_upper) // NOLINT
{
  // clang-format off
  const tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1, 4, 6}, 
     {0, 5, 7}, 
     {0, 0, 8}});
  const tracking::math::Matrix<float32, 3, 4> bMat(
    {{1,  2,  3,  4}, 
     {5,  6,  7,  8}, 
     {9, 10, 11, 12}});
  const tracking::math::Matrix<float32, 3, 4> expMat(
    {{-3.450, -3.30, -3.150, -3.00}, 
     {-0.575, -0.55, -0.525, -0.50}, 
     { 1.125,  1.25,  1.375,  1.50}});
  // clang-format on

  // call UUT
  auto resMat = triuMat.solve(bMat);

  for (auto row = 0; row < 3; ++row)
  {
    for (auto col = row; col < 4; ++col)
    {
      EXPECT_FLOAT_EQ(expMat(row, col), resMat(row, col));
    }
  }
}

TEST(TriangularMatrix, isUnitUpperTriangular_false) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1, 2, 3}, 
     {0, 1.01, 5}, 
     {0, 0, 0.9999999}});
  // clang-format on

  // call UUT
  auto result = triuMat.isUnitUpperTriangular();

  EXPECT_FALSE(result);
}

TEST(TriangularMatrix, isUnitUpperTriangular_true) // NOLINT
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> triuMat(
    {{1, 2, 3}, 
     {0, 1, 5}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  auto result = triuMat.isUnitUpperTriangular();

  EXPECT_TRUE(result);
}
