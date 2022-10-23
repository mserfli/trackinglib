#include "gtest/gtest.h"
#include "trackingLib/math/linalg/diagonal_matrix.hpp"


// instatiate all templates for full coverage report
template class tracking::math::DiagonalMatrix<float32, 3>;
template void tracking::math::DiagonalMatrix<float32, 3>::setBlock<2, 2, 0, 0>(const DiagonalMatrix<float32, 2>&);

TEST(DiagonalMatrix, setBlock_topLeft) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 1, 0}, 
     {0, 0, 1}});
  const tracking::math::DiagonalMatrix<float32, 2> diagBlockMat(
    {{2, 0},  
     {0, 3}});
  const tracking::math::DiagonalMatrix<float32, 3> expMat(
    {{2, 0, 0}, 
     {0, 3, 0}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  diagMat.setBlock<2, 2, 0, 0>(diagBlockMat);

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, setBlock_bottomRight) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 1, 0}, 
     {0, 0, 1}});
  const tracking::math::DiagonalMatrix<float32, 2> diagBlockMat(
    {{2, 0},  
     {0, 3}});
  const tracking::math::DiagonalMatrix<float32, 3> expMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  // clang-format on

  // call UUT
  diagMat.setBlock<2, 2, 0, 1>(diagBlockMat);

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, op_mul_vec) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const tracking::math::Vector<float32, 3> vec(
    {4, 5, 6});
  const tracking::math::Vector<float32, 3> expMat(
    {4,  10,  18});
  // clang-format on

  // call UUT
  auto resMat = diagMat * vec;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_mat) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const tracking::math::Matrix<float32, 3, 4> mat(
    {{1,  2,  3,  4}, 
     {5,  6,  7,  8}, 
     {9, 10, 11, 12}});
  const tracking::math::Matrix<float32, 3, 4> expMat(
    {{1,   2,  3,  4}, 
     {10, 12, 14, 16}, 
     {27, 30, 33, 36}});
  // clang-format on

  // call UUT
  auto resMat = diagMat * mat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_lowerTria) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const tracking::math::TriangularMatrix<float32, 3, true> mat(
    {{1,  0,  0}, 
     {4,  5,  0}, 
     {6,  7,  8}});
  const tracking::math::Matrix<float32, 3, 3> expMat(
    {{ 1,  0,  0}, 
     { 8, 10,  0}, 
     {18, 21, 24}});
  // clang-format on

  // call UUT
  auto resMat = diagMat * mat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_upperTria) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const tracking::math::TriangularMatrix<float32, 3, false> mat(
    {{1,  2,  3}, 
     {0,  5,  6}, 
     {0,  0,  8}});
  const tracking::math::Matrix<float32, 3, 3> expMat(
    {{ 1,  2,  3}, 
     { 0, 10, 12}, 
     { 0,  0, 24}});
  // clang-format on

  // call UUT
  auto resMat = diagMat * mat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_diag) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const tracking::math::DiagonalMatrix<float32, 3> diagMatOther(
    {{4, 0, 0}, 
     {0, 5, 0}, 
     {0, 0, 6}});
  const tracking::math::DiagonalMatrix<float32, 3> expMat(
    {{4,  0,  0}, 
     {0, 10,  0}, 
     {0,  0, 18}});
  // clang-format on

  // call UUT
  auto resMat = diagMat * diagMatOther;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_diag_inplace) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const tracking::math::DiagonalMatrix<float32, 3> diagMatOther(
    {{4, 0, 0}, 
     {0, 5, 0}, 
     {0, 0, 6}});
  const tracking::math::DiagonalMatrix<float32, 3> expMat(
    {{4,  0,  0}, 
     {0, 10,  0}, 
     {0,  0, 18}});
  // clang-format on

  // call UUT
  diagMat *= diagMatOther;

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, op_mul_scal) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const float32 scal = 3;
  const tracking::math::DiagonalMatrix<float32, 3> expMat(
    {{3, 0, 0}, 
     {0, 6, 0}, 
     {0, 0, 9}});
  // clang-format on

  // call UUT
  auto resMat = diagMat * scal;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_scal_inplace) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 3}});
  const float32 scal = 3;
  const tracking::math::DiagonalMatrix<float32, 3> expMat(
    {{3, 0, 0}, 
     {0, 6, 0}, 
     {0, 0, 9}});
  // clang-format on

  // call UUT
  diagMat *= scal;

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, inverse) // NOLINT
{
  // clang-format off
  const tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 4}});
  const tracking::math::DiagonalMatrix<float32, 3> expMat(
    {{1, 0,   0}, 
     {0, 0.5, 0}, 
     {0, 0,   0.25}});
  // clang-format on

  // call UUT
  auto invMat = diagMat.inverse();

  EXPECT_EQ(expMat._data, invMat._data);
}

TEST(DiagonalMatrix, inverse_inplace) // NOLINT
{
  // clang-format off
  tracking::math::DiagonalMatrix<float32, 3> diagMat(
    {{1, 0, 0}, 
     {0, 2, 0}, 
     {0, 0, 4}});
  const tracking::math::DiagonalMatrix<float32, 3> expMat(
    {{1, 0,   0}, 
     {0, 0.5, 0}, 
     {0, 0,   0.25}});
  // clang-format on

  // call UUT
  diagMat.inverse();

  EXPECT_EQ(expMat._data, diagMat._data);
}

