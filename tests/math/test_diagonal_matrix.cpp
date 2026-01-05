#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"   // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/diagonal_matrix.hpp"                    // IWYU pragma: keep
#include <limits>

using namespace tracking::math;

TEST(DiagonalMatrix, ctor_default) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = DiagonalMatrix<float32, 3>;
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
  });
  // clang-format on

  // call UUT
  const auto diagMat = DiagonalMatrix{};

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, Identity) // NOLINT
{
  // clang-format off
  using DiagonalMatrix = DiagonalMatrix<float32, 3>;
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  using DiagonalMatrix = DiagonalMatrix<float32, 3>;
  DiagonalMatrix diagMat{}; 
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 1, 0}, 
    {0, 0, 1}
  });
  const auto diagBlockMat = conversions::DiagonalFromList<float32, 2>({
    {2, 0},  
    {0, 3}
  });
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 1, 0}, 
    {0, 0, 1}
  });
  const auto diagBlockMat = conversions::DiagonalFromList<float32, 2>({
    {2, 0},  
    {0, 3}
  });
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 4}
  });
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 4}
  });
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto vec = conversions::VectorFromList<float32, 3>({
    4, 5, 6
  });
  const auto expMat = conversions::VectorFromList<float32, 3>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto mat = conversions::MatrixFromList<float32, 3, 4, true>({
    {1,  2,  3,  4}, 
    {5,  6,  7,  8}, 
    {9, 10, 11, 12}
  });
  const auto expMat = conversions::MatrixFromList<float32, 3, 4, true>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto mat = conversions::MatrixFromList<float32, 3, 3, true>({
    {1, 2, 3}, 
    {4, 5, 6}, 
    {7, 8, 9}
  });
  const auto expMat = conversions::MatrixFromList<float32, 3, 3, true>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto trilMat = conversions::TriangularFromList<float32, 3, true, true>({
    {1, 0, 0}, 
    {4, 5, 0}, 
    {6, 7, 8}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, true, true>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto triuMat = conversions::TriangularFromList<float32, 3, false, true>({
    {1, 2, 3}, 
    {0, 5, 6}, 
    {0, 0, 8}
  });
  const auto expMat = conversions::TriangularFromList<float32, 3, false, true>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto diagMatOther = conversions::DiagonalFromList<float32, 3>({
    {4, 0, 0}, 
    {0, 5, 0}, 
    {0, 0, 6}
  });
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const auto diagMatOther = conversions::DiagonalFromList<float32, 3>({
    {4, 0, 0}, 
    {0, 5, 0}, 
    {0, 0, 6}
  });
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const float32 scal = 3;
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0}, 
    {0, 2, 0}, 
    {0, 0, 3}
  });
  const float32 scal = 3;
  const auto expMat = conversions::DiagonalFromList<float32, 3>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
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
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {val, 0,   0}, 
    {  0, 0,   0}, 
    {  0, 0, val}
  });
  // clang-format on

  // call UUT
  auto result = diagMat.isPositiveDefinite();

  EXPECT_EQ(diagMat.at_unsafe(0), diagMat.at_unsafe(2));
  EXPECT_FALSE(result);
}
