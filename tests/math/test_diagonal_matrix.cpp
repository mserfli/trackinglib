#include "gtest/gtest.h"
#include "trackingLib/math/linalg/diagonal_matrix.hpp" // IWYU pragma: keep
#include <limits>

TEST(DiagonalMatrix, ctor_default) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto expMat = DiagonalMatrix::FromList({
    {0, 0, 0}, 
    {0, 0, 0}, 
    {0, 0, 0}
  });
  // clang-format on

  // call UUT
  const auto diagMat = DiagonalMatrix{};

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, ctor_square) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto mat = tracking::math::SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3}, 
    {4, 5, 6}, 
    {7, 8, 9}
  });
  const auto expMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 5, 0}, 
    {0, 0, 9}
  });
  // clang-format on

  // call UUT
  const auto diagMat = DiagonalMatrix::FromMatrix(mat);

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, ctor_list) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto expMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 5, 0}, 
    {0, 0, 9}
  });
  // clang-format on

  // call UUT
  const auto diagMat = DiagonalMatrix::FromList({1, 5, 9});

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, Identity) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto expMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 1, 0}, 
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  const auto diagMat{DiagonalMatrix::Identity()};

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, setIdentity) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  DiagonalMatrix diagMat{}; 
  const auto expMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 1, 0}, 
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  diagMat.setIdentity();

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, setBlock_topLeft) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 1, 0}, 
    {0, 0, 1}
  });
  const auto diagBlockMat = tracking::math::DiagonalMatrix<float32, 2>::FromList({
    {2, 0},  
    {0, 3}
  });
  const auto expMat = DiagonalMatrix::FromList({
    {2, 0, 0}, 
    {0, 3, 0}, 
    {0, 0, 1}
  });
  // clang-format on

  // call UUT
  diagMat.setBlock<2, 2, 0, 0>(diagBlockMat);

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, setBlock_bottomRight) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 1, 0}, 
    {0, 0, 1}
  });
  const auto diagBlockMat = tracking::math::DiagonalMatrix<float32, 2>::FromList({
    {2, 0},  
    {0, 3}
  });
  const auto expMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  // clang-format on

  // call UUT
  diagMat.setBlock<2, 2, 0, 1>(diagBlockMat);

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, inverse) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 4}
  });
  const auto expMat = DiagonalMatrix::FromList({
    {1, 0,   0}, 
    {0, 0.5, 0}, 
    {0, 0,   0.25}
  });
  // clang-format on

  // call UUT
  auto invMat = diagMat.inverse();

  EXPECT_EQ(expMat._data, invMat._data);
}

TEST(DiagonalMatrix, inverse_inplace) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 4}
  });
  const auto expMat = DiagonalMatrix::FromList({
    {1, 0,   0}, 
    {0, 0.5, 0}, 
    {0, 0,   0.25}
  }); // clang-format on

  // call UUT
  diagMat.inverse();

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, op_mul_rhs_vec) // NOLINT
{
  // clang-format off
  const auto diagMat = tracking::math::DiagonalMatrix<float32, 3>::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto vec = tracking::math::Vector<float32, 3>::FromList({
    4, 5, 6
  });
  const auto expMat = tracking::math::Vector<float32, 3>::FromList({
    4,  10,  18
  });
  // clang-format on

  // call UUT
  auto resMat = diagMat * vec;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_rhs_mat) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  using Matrix34 = tracking::math::Matrix<float32, 3, 4, true>;
  const auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto mat = Matrix34::FromList({
    {1,  2,  3,  4}, 
    {5,  6,  7,  8}, 
    {9, 10, 11, 12}
  });
  const auto expMat = Matrix34::FromList({
    {1,  2,  3,  4}, 
    {10, 12, 14, 16}, 
    {27, 30, 33, 36}
  });
  // clang-format on

  // call UUT
  auto resMat = diagMat * mat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_lhs_mat) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  using Matrix33 = tracking::math::Matrix<float32, 3, 3, true>;
  const auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto mat = Matrix33::FromList({
    {1, 2, 3}, 
    {4, 5, 6}, 
    {7, 8, 9}
  });
  const auto expMat = Matrix33::FromList({
    {1, 4, 9}, 
    {4, 10, 18}, 
    {7, 16, 27}
  });
  // clang-format on

  // call UUT
  auto resMat = mat * diagMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_rhs_lowerTria) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  using LowerTriangularMatrix = tracking::math::TriangularMatrix<float32, 3, true, true>;
  const auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto trilMat = LowerTriangularMatrix::FromList({
    {1, 0, 0}, 
    {4, 5, 0}, 
    {6, 7, 8}
  });
  const auto expMat = LowerTriangularMatrix::FromList({
    {1, 0, 0}, 
    {8, 10, 0}, 
    {18, 21, 24}
  });
  // clang-format on

  // call UUT
  auto resMat = diagMat * trilMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_rhs_upperTria) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  using UpperTriangularMatrix = tracking::math::TriangularMatrix<float32, 3, false, true>;
  const auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto triuMat = UpperTriangularMatrix::FromList({
    {1, 2, 3}, 
    {0, 5, 6}, 
    {0, 0, 8}
  });
  const auto expMat = UpperTriangularMatrix::FromList({
    {1, 2, 3}, 
    {0, 10, 12}, 
    {0, 0, 24}
  });
  // clang-format on

  // call UUT
  auto resMat = diagMat * triuMat;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_diag) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto diagMatOther = DiagonalMatrix::FromList({
    {4, 0, 0}, 
    {0, 5, 0}, 
    {0, 0, 6}
  });
  const auto expMat = DiagonalMatrix::FromList({
    {4, 0, 0}, 
    {0, 10, 0}, 
    {0, 0, 18}
  });
  // clang-format on

  // call UUT
  auto resMat = diagMat * diagMatOther;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_diag_inplace) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto diagMatOther = DiagonalMatrix::FromList({
    {4, 0, 0}, 
    {0, 5, 0}, 
    {0, 0, 6}
  });
  const auto expMat = DiagonalMatrix::FromList({
    {4, 0, 0}, 
    {0, 10, 0}, 
    {0, 0, 18}
  });
  // clang-format on

  // call UUT
  diagMat *= diagMatOther;

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, op_mul_scal) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const float32 scal = 3;
  const auto expMat = DiagonalMatrix::FromList({
    {3, 0, 0}, 
    {0, 6, 0}, 
    {0, 0, 9}
  });
  // clang-format on

  // call UUT
  auto resMat = diagMat * scal;

  EXPECT_EQ(expMat._data, resMat._data);
}

TEST(DiagonalMatrix, op_mul_scal_inplace) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const float32 scal = 3;
  const auto expMat = DiagonalMatrix::FromList({
    {3, 0, 0}, 
    {0, 6, 0}, 
    {0, 0, 9}
  });
  // clang-format on

  // call UUT
  diagMat *= scal;

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, isPositiveDefinite_true) // NOLINT
{
  // clang-format off
  const auto val = std::numeric_limits<float32>::min();
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto diagMat = DiagonalMatrix::FromList({
    {val,   0,   0}, 
    {  0, val,   0}, 
    {  0,   0, val}
  });
  // clang-format on

  // call UUT
  auto result = diagMat.isPositiveDefinite();

  EXPECT_TRUE(result);
}

TEST(DiagonalMatrix, isPositiveDefinite_false) // NOLINT
{
  // clang-format off
  const auto val = std::numeric_limits<float32>::min();
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto diagMat = DiagonalMatrix::FromList({
    {val, 0,   0}, 
    {  0, 0,   0}, 
    {  0, 0, val}
  });
  // clang-format on

  // call UUT
  auto result = diagMat.isPositiveDefinite();

  EXPECT_FALSE(result);
}

TEST(DiagonalMatrix, print) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = tracking::math::DiagonalMatrix<float32, 3>;
  const auto diagMat = DiagonalMatrix::FromList({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  // clang-format on

  // call UUT
  diagMat.print();
}
