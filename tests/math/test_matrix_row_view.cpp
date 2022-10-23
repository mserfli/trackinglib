#include "gtest/gtest.h"

#include "trackingLib/math/linalg/matrix_row_view.hpp"
#include "trackingLib/math/linalg/vector.hpp"

TEST(MatrixRowView, mul_lhs) // NOLINT
{
  // testing Vector * MatrixRowView
  const tracking::math::Matrix<float32, 3, 3>        mat{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}};
  const tracking::math::MatrixRowView<float32, 3, 3> rowView(mat, 2, 0, 2);
  const tracking::math::Vector<float32, 3>           lhs{{1, 2, 3}};

  // call UUT
  auto res = lhs * rowView;

  EXPECT_FLOAT_EQ(res, 1 * 7 + 2 * 8 + 3 * 9);
}

TEST(MatrixRowView, mul_lhs_ranged) // NOLINT
{
  // testing Vector * MatrixRowView
  const tracking::math::Matrix<float32, 3, 3>        mat{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}};
  const tracking::math::MatrixRowView<float32, 3, 3> rowView(mat, 2, 1, 2);
  const tracking::math::Vector<float32, 2>           lhs{{1, 2}};

  // call UUT
  auto res = lhs * rowView;

  EXPECT_FLOAT_EQ(res, 1 * 8 + 2 * 9);
}

TEST(MatrixRowView, mul_rhs) // NOLINT
{
  // testing MatrixRowView * Matrix
  const tracking::math::Matrix<float32, 4, 3>        mat{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}}};
  const tracking::math::Matrix<float32, 3, 2>        rhs{{1, 2}, {3, 4}, {5, 6}};
  const tracking::math::MatrixRowView<float32, 4, 3> rowView(mat, 2, 0, 2);

  // call UUT
  auto res = rowView * rhs;

  EXPECT_FLOAT_EQ(res(0,0), 7*1+8*3+9*5);
  EXPECT_FLOAT_EQ(res(0,1), 7*2+8*4+9*6);
}

TEST(MatrixRowView, mul_rhs_ranged) // NOLINT
{
  // testing MatrixRowView * Matrix
  const tracking::math::Matrix<float32, 4, 3>        mat{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}}};
  const tracking::math::Matrix<float32, 2, 2>        rhs{{1, 2}, {3, 4}};
  const tracking::math::MatrixRowView<float32, 4, 3> rowView(mat, 2, 1, 2);

  // call UUT
  auto res = rowView * rhs;

  EXPECT_FLOAT_EQ(res(0,0), 8*1+9*3);
  EXPECT_FLOAT_EQ(res(0,1), 8*2+9*4);
}
