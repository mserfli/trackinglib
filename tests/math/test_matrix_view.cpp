#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"
#include "trackingLib/math/linalg/matrix_view.hpp" // IWYU pragma: keep

using namespace tracking::math;

TEST(MatrixView, ctor_view) // NOLINT
{
  // testing MatrixView on existing matrix
  // clang-format off
  const auto mat{conversions::MatrixFromList<sint32, 3, 3, true>({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  })};
  // clang-format on

  // View of the top-left 2x2 block
  const MatrixView<sint32, 3, 3> view(mat, 0, 0, 1, 1);

  EXPECT_EQ(view.getRowCount(), 2);
  EXPECT_EQ(view.getColCount(), 2);
  EXPECT_EQ(view(0, 0), 1);
  EXPECT_EQ(view(0, 1), 2);
  EXPECT_EQ(view(1, 0), 4);
  EXPECT_EQ(view(1, 1), 5);
}

TEST(MatrixView, add_views) // NOLINT
{
  // clang-format off
  const auto mat{conversions::MatrixFromList<sint32, 3, 3, true>({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  })};
  // clang-format on

  // View 1: top-left 2x2
  const MatrixView<sint32, 3, 3> view1(mat, 0, 0, 1, 1);
  // View 2: bottom-right 2x2
  const MatrixView<sint32, 3, 3> view2(mat, 1, 1, 2, 2);

  // Result should be 2x2 matrix (stored in 3x3)
  auto res = view1 + view2;

  // res is Matrix<sint32, 3, 3>
  static_assert(std::is_same<decltype(res), Matrix<sint32, 3, 3, true>>::value);

  // view1: {{1, 2}, {4, 5}}
  // view2: {{5, 6}, {8, 9}}
  // sum:   {{6, 8}, {12, 14}}
  EXPECT_EQ(res.at_unsafe(0, 0), 6);
  EXPECT_EQ(res.at_unsafe(0, 1), 8);
  EXPECT_EQ(res.at_unsafe(1, 0), 12);
  EXPECT_EQ(res.at_unsafe(1, 1), 14);
}

TEST(MatrixView, mul_views) // NOLINT
{
  // clang-format off
  const auto mat{conversions::MatrixFromList<sint32, 3, 3, true>({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  })};
  // clang-format on

  // View 1: top-left 2x2
  const MatrixView<sint32, 3, 3> view1(mat, 0, 0, 1, 1);
  // View 2: bottom-right 2x2
  const MatrixView<sint32, 3, 3> view2(mat, 1, 1, 2, 2);

  // Result should be 2x2 matrix
  auto res = view1 * view2;

  // view1: {{1, 2}, {4, 5}}
  // view2: {{5, 6}, {8, 9}}
  // prod:  {{1*5+2*8, 1*6+2*9}, {4*5+5*8, 4*6+5*9}}
  //        {{5+16, 6+18}, {20+40, 24+45}}
  //        {{21, 24}, {60, 69}}

  EXPECT_EQ(res.at_unsafe(0, 0), 21);
  EXPECT_EQ(res.at_unsafe(0, 1), 24);
  EXPECT_EQ(res.at_unsafe(1, 0), 60);
  EXPECT_EQ(res.at_unsafe(1, 1), 69);
}

TEST(MatrixView, scalar_ops) // NOLINT
{
  // clang-format off
  const auto mat{conversions::MatrixFromList<sint32, 3, 3, true>({
    {1, 2, 3}, 
    {4, 5, 6}, 
    {7, 8, 9}
  })};
  // clang-format on
  const MatrixView<sint32, 3, 3> view(mat, 0, 0, 1, 1);

  auto res = view * 2;
  EXPECT_EQ(res.at_unsafe(0, 0), 2);
  EXPECT_EQ(res.at_unsafe(0, 1), 4);
  EXPECT_EQ(res.at_unsafe(1, 0), 8);
  EXPECT_EQ(res.at_unsafe(1, 1), 10);
}
