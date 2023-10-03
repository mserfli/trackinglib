#include "gtest/gtest.h"

#include "trackingLib/math/linalg/matrix_row_view.hpp"
#include "trackingLib/math/linalg/vector.hpp"

TEST(MatrixRowView, mul_rhs) // NOLINT
{
  // testing MatrixRowView * Matrix
  // clang-format off
  const auto mat{tracking::math::Matrix<sint32, 4, 3, true>::FromList({
    {1, 2, 3}, 
    {4, 5, 6}, 
    {7, 8, 9},
    {10,11,12},
  })};
  const auto rhs{tracking::math::Matrix<sint32, 3, 2, true>::FromList({
    {1, 2}, 
    {3, 4}, 
    {5, 6},
  })};
  const tracking::math::MatrixRowView<sint32, 4, 3, true> rowView(mat, 2, 0, 2);
  // clang-format on

  // calc expected result
  tracking::math::Matrix<sint32, 1, 3, true> row;
  row.setBlock<4, 3, 1, 3, 2, 0, true, 0, 0>(mat);
  const auto resExp = row * rhs;

  // call UUT
  auto res = rowView * rhs;

  for (auto row = 0; row < 1; row++)
  {
    for (auto col = 0; col < 2; col++)
    {
      EXPECT_EQ(res.at_unsafe(row, col), resExp.at_unsafe(row, col));
    }
  }
}

TEST(MatrixRowView, mul_rhs_ranged) // NOLINT
{
  // testing MatrixRowView * Matrix
  // clang-format off
  const auto mat{tracking::math::Matrix<sint32, 4, 3, true>::FromList({
    {1, 2, 3}, 
    {4, 5, 6}, 
    {7, 8, 9},
    {10,11,12},
  })};
  const auto rhs{tracking::math::Matrix<sint32, 2, 2, true>::FromList({
    {1, 2}, 
    {3, 4}, 
  })};
  const tracking::math::MatrixRowView<sint32, 4, 3, true> rowView(mat, 2, 1, 2);
  // clang-format on

  // calc expected result
  tracking::math::Matrix<sint32, 1, 2, true> row;
  row.setBlock<4, 3, 1, 2, 2, 1, true, 0, 0>(mat);
  const auto resExp = row * rhs;

  // call UUT
  auto res = rowView * rhs;

  for (auto row = 0; row < 1; row++)
  {
    for (auto col = 0; col < 2; col++)
    {
      EXPECT_EQ(res.at_unsafe(row, col), resExp.at_unsafe(row, col));
    }
  }
}

TEST(MatrixRowView, dot_colView) // NOLINT
{
  // testing MatrixRowView * MatrixColView
  // clang-format off
  const auto mat{tracking::math::Matrix<sint32, 4, 3, true>::FromList({
    {1, 2, 3}, 
    {4, 5, 6}, 
    {7, 8, 9},
    {10,11,12},
  })};
  const tracking::math::MatrixRowView<sint32, 4, 3, true> rowView(mat, 2, 1, 2);
  const tracking::math::MatrixColumnView<sint32, 4, 3, true> colView(mat, 2, 1, 2);
  // clang-format on

  // calc expected result
  tracking::math::Matrix<sint32, 1, 2, true> row;
  tracking::math::Matrix<sint32, 2, 1, true> col;
  row.setBlock<4, 3, 1, 2, 2, 1, true, 0, 0>(mat);
  col.setBlock<4, 3, 2, 1, 1, 2, true, 0, 0>(mat);
  const auto resExp = row * col;

  // call UUT
  auto res = rowView * colView;

  EXPECT_EQ(res, resExp.at_unsafe(0,0));
}
