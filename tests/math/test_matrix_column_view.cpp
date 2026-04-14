#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"
#include "trackingLib/math/linalg/matrix_column_view.hpp" // IWYU pragma: keep

using namespace tracking::math;

TEST(MatrixColumnView, mul_lhs) // NOLINT
{
  // testing Matrix * MatrixColumnView
  // clang-format off
  const auto mat{Matrix<sint32, 3, 3, true>::FromList({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  })};
  // clang-format on
  // the column view
  const MatrixColumnView<sint32, 3, 3, true> colView(mat, 2, 0, 2);

  // a lhs matrix
  const auto lhs = mat;

  // calc expected result
  Matrix<sint32, 3, 1, true> col;
  col.setBlock<3, 3, 3, 1, 0, 2, true, 0, 0>(mat);
  const auto resExp = lhs * col;

  // call UUT
  auto res = lhs * colView;

  EXPECT_EQ(res, resExp);
}

TEST(MatrixColumnView, mul_lhs_ranged) // NOLINT
{
  // testing Matrix * MatrixColumnView
  // clang-format off
  const auto mat{Matrix<sint32, 3, 3, true>::FromList({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  })};
  // clang-format on
  // the column view
  const MatrixColumnView<sint32, 3, 3, true> colView(mat, 2, 1, 2);

  // a lhs matrix
  // clang-format off
  const auto lhs{Matrix<sint32, 3, 2, true>::FromList({
    {1, 2}, 
    {3, 4}, 
    {5, 6}}
  )};
  // clang-format on

  // calc expected result
  Matrix<sint32, 2, 1, true> col;
  col.setBlock<3, 3, 2, 1, 1, 2, true, 0, 0>(mat);
  const auto resExp = lhs * col;

  // call UUT
  auto res = lhs * colView;

  EXPECT_EQ(res, resExp);
}

TEST(MatrixColumnView, mul_rhs) // NOLINT
{
  // testing MatrixColumnView * Vector
  // clang-format off
  const auto mat{Matrix<sint32, 4, 3, true>::FromList({
    { 1,  2,  3},
    { 4,  5,  6},
    { 7,  8,  9},
    {10, 11, 12}
  })};
  const auto rhs{Vector<sint32, 4>::FromList({
    1, 2, 3, 4}
  )};
  // clang-format on
  const MatrixColumnView<sint32, 4, 3, true> colView(mat, 2, 0, 3);

  // calc expected result
  Matrix<sint32, 4, 1, true> col;
  col.setBlock<4, 3, 4, 1, 0, 2, true, 0, 0>(mat);
  const auto resExp = (col.transpose() * rhs).at_unsafe(0, 0);

  // call UUT
  auto res = colView * rhs;

  EXPECT_EQ(res, resExp);
}

TEST(MatrixColumnView, mul_rhs_ranged) // NOLINT
{
  // testing MatrixColumnView * Vector
  // clang-format off
  const auto mat{Matrix<sint32, 4, 3, true>::FromList({
    { 1,  2,  3},
    { 4,  5,  6},
    { 7,  8,  9},
    {10, 11, 12}
  })};
  const auto rhs{Vector<sint32, 2>::FromList({
    1, 2
  })};
  // clang-format on
  const MatrixColumnView<sint32, 4, 3, true> colView(mat, 2, 1, 2);

  // calc expected result
  Matrix<sint32, 2, 1, true> col;
  col.setBlock<4, 3, 2, 1, 1, 2, true, 0, 0>(mat);
  const auto resExp = (col.transpose() * rhs).at_unsafe(0, 0);

  // call UUT
  auto res = colView * rhs;

  EXPECT_EQ(res, resExp);
}

TEST(MatrixColumnView, mul_rhsview) // NOLINT
{
  // testing MatrixColumnView * MatrixColumnView
  // clang-format off
  const auto mat{Matrix<sint32, 4, 3, true>::FromList({
    { 1,  2,  3},
    { 4,  5,  6},
    { 7,  8,  9},
    {10, 11, 12}
  })};
  const auto rhs{Vector<sint32, 4>::FromList({
    1, 2, 3, 4
  })};
  // clang-format on
  const MatrixColumnView<sint32, 4, 3, true> colView(mat, 2, 1, 2);
  const MatrixColumnView<sint32, 4, 1, true> rhsView(rhs, 0, 1, 2);

  // calc expected result
  Matrix<sint32, 2, 1, true> col1;
  col1.setBlock<4, 3, 2, 1, 1, 2, true, 0, 0>(mat);
  Matrix<sint32, 2, 1, true> col2;
  col2.setBlock<4, 1, 2, 1, 1, 0, true, 0, 0>(rhs);
  const auto resExp = (col1.transpose() * col2).at_unsafe(0, 0);

  // call UUT
  auto res = colView * rhsView;

  EXPECT_EQ(res, resExp);
}
