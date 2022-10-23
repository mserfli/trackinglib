#include "gtest/gtest.h"

#include "trackingLib/math/linalg/matrix_column_view.hpp"
#include "trackingLib/math/linalg/vector.hpp"

TEST(MatrixColumnView, mul_lhs)
{
  // testing Matrix * MatrixColumnView
  const tracking::math::Matrix<float32, 3, 3>           lhs{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}};
  const tracking::math::MatrixColumnView<float32, 3, 3> colView(lhs, 2, 0, 2);

  // call UUT
  auto res = lhs * colView;

  EXPECT_FLOAT_EQ(res[0], 1 * 3 + 2 * 6 + 3 * 9);
  EXPECT_FLOAT_EQ(res[1], 4 * 3 + 5 * 6 + 6 * 9);
  EXPECT_FLOAT_EQ(res[2], 7 * 3 + 8 * 6 + 9 * 9);
}

TEST(MatrixColumnView, mul_lhs_ranged)
{
  // testing Matrix * MatrixColumnView
  const tracking::math::Matrix<float32, 3, 3>           mat{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}};
  const tracking::math::MatrixColumnView<float32, 3, 3> colView(mat, 2, 1, 2);
  const tracking::math::Matrix<float32, 3, 2>           lhs{{{1, 2}, {3, 4}, {5, 6}}};

  // call UUT
  auto res = lhs * colView;

  EXPECT_FLOAT_EQ(res[0], 1 * 6 + 2 * 9);
  EXPECT_FLOAT_EQ(res[1], 3 * 6 + 4 * 9);
  EXPECT_FLOAT_EQ(res[2], 5 * 6 + 6 * 9);
}

TEST(MatrixColumnView, mul_rhs)
{
  // testing MatrixColumnView * Vector
  const tracking::math::Matrix<float32, 4, 3>           mat{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}}};
  const tracking::math::Vector<float32, 4>              rhs{{1, 2, 3, 4}};
  const tracking::math::MatrixColumnView<float32, 4, 3> colView(mat, 2, 0, 3);

  // call UUT
  auto res = colView * rhs;

  EXPECT_FLOAT_EQ(res, 3 * 1 + 6 * 2 + 9 * 3 + 12 * 4);
}

TEST(MatrixColumnView, mul_rhs_ranged)
{
  // testing MatrixColumnView * Vector
  const tracking::math::Matrix<float32, 4, 3>           mat{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}}};
  const tracking::math::Vector<float32, 2>              rhs{{1, 2}};
  const tracking::math::MatrixColumnView<float32, 4, 3> colView(mat, 2, 1, 2);

  // call UUT
  auto res = colView * rhs;

  EXPECT_FLOAT_EQ(res, 6 * 1 + 9 * 2);
}
