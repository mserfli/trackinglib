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

TEST(DiagonalMatrix, FromList_Flat__Success) // NOLINT
{
  const auto result = DiagonalMatrix<sint32, 3>::FromList({1, 2, 3});

  EXPECT_EQ(result.at_unsafe(0), 1);
  EXPECT_EQ(result.at_unsafe(1), 2);
  EXPECT_EQ(result.at_unsafe(2), 3);
}

TEST(DiagonalMatrix, FromList_Nested__Success) // NOLINT
{
  // clang-format off
  const auto result = DiagonalMatrix<sint32, 3>::FromList({
      {1, 0, 0},
      {0, 2, 0},
      {0, 0, 3},
  });
  // clang-format on

  EXPECT_EQ(result.at_unsafe(0), 1);
  EXPECT_EQ(result.at_unsafe(1), 2);
  EXPECT_EQ(result.at_unsafe(2), 3);
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

// ============================================================================
// Trace and Determinant Tests
// ============================================================================

TEST(DiagonalMatrix, trace_2x2__Success) // NOLINT
{
  // Create a 2x2 diagonal matrix
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 2>({
    {1, 0},
    {0, 2}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.trace();

  // Expected: 1 + 2 = 3
  EXPECT_FLOAT_EQ(result, 3.0F);
}

TEST(DiagonalMatrix, trace_3x3__Success) // NOLINT
{
  // Create a 3x3 diagonal matrix
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.trace();

  // Expected: 1 + 2 + 3 = 6
  EXPECT_FLOAT_EQ(result, 6.0F);
}

TEST(DiagonalMatrix, trace_4x4__Success) // NOLINT
{
  // Create a 4x4 diagonal matrix
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 4>({
    {1, 0, 0, 0},
    {0, 2, 0, 0},
    {0, 0, 3, 0},
    {0, 0, 0, 4}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.trace();

  // Expected: 1 + 2 + 3 + 4 = 10
  EXPECT_FLOAT_EQ(result, 10.0F);
}

TEST(DiagonalMatrix, trace_IdentityMatrix__Success) // NOLINT
{
  const auto identity = DiagonalMatrix<float32, 3>::Identity();

  // call UUT
  const auto result = identity.trace();

  // Expected: 1 + 1 + 1 = 3 (matrix size)
  EXPECT_FLOAT_EQ(result, 3.0F);
}

TEST(DiagonalMatrix, trace_ZeroMatrix__Success) // NOLINT
{
  const auto zeroMat = DiagonalMatrix<float32, 3>{};

  // call UUT
  const auto result = zeroMat.trace();

  // Expected: 0 + 0 + 0 = 0
  EXPECT_FLOAT_EQ(result, 0.0F);
}

TEST(DiagonalMatrix, trace_Double__Success) // NOLINT
{
  // Create a 3x3 diagonal matrix with double precision
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float64, 3>({
    {1.0, 0.0, 0.0},
    {0.0, 2.0, 0.0},
    {0.0, 0.0, 3.0}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.trace();

  // Expected: 1.0 + 2.0 + 3.0 = 6.0
  EXPECT_DOUBLE_EQ(result, 6.0);
}

TEST(DiagonalMatrix, trace_1x1_Consistency__Success) // NOLINT
{
  // Create a 1x1 diagonal matrix
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 1>({
    {5}
  });
  // clang-format on

  // call UUT
  const auto traceResult = diagMat.trace();
  const auto detResult   = diagMat.determinant();

  // For 1x1 matrices, trace should equal determinant
  EXPECT_FLOAT_EQ(traceResult, detResult);
  EXPECT_FLOAT_EQ(traceResult, 5.0F);
}

TEST(DiagonalMatrix, determinant_2x2__Success) // NOLINT
{
  // Create a 2x2 diagonal matrix
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 2>({
    {1, 0},
    {0, 2}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.determinant();

  // Expected: 1 * 2 = 2 (product of diagonal elements)
  EXPECT_FLOAT_EQ(result, 2.0F);
}

TEST(DiagonalMatrix, determinant_3x3__Success) // NOLINT
{
  // Create a 3x3 diagonal matrix
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.determinant();

  // Expected: 1 * 2 * 3 = 6 (product of diagonal elements)
  EXPECT_FLOAT_EQ(result, 6.0F);
}

TEST(DiagonalMatrix, determinant_IdentityMatrix__Success) // NOLINT
{
  const auto identity = DiagonalMatrix<float32, 3>::Identity();

  // call UUT
  const auto result = identity.determinant();

  // Expected: 1 (identity matrix determinant is always 1)
  EXPECT_FLOAT_EQ(result, 1.0F);
}

TEST(DiagonalMatrix, determinant_SingularMatrix__Success) // NOLINT
{
  // Create a singular diagonal matrix (zero on diagonal)
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0},
    {0, 0, 0},
    {0, 0, 3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.determinant();

  // Expected: 1 * 0 * 3 = 0 (singular matrix)
  EXPECT_FLOAT_EQ(result, 0.0F);
}

TEST(DiagonalMatrix, determinant_Double__Success) // NOLINT
{
  // Create a 3x3 diagonal matrix with double precision
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float64, 3>({
    {1.0, 0.0, 0.0},
    {0.0, 2.0, 0.0},
    {0.0, 0.0, 3.0}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.determinant();

  // Expected: 1.0 * 2.0 * 3.0 = 6.0
  EXPECT_DOUBLE_EQ(result, 6.0);
}

TEST(DiagonalMatrix, determinant_NegativeElements__Success) // NOLINT
{
  // Create a diagonal matrix with negative elements
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {-1, 0,  0},
    { 0, 2,  0},
    { 0, 0, -3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.determinant();

  // Expected: (-1) * 2 * (-3) = 6 (product of diagonal elements)
  EXPECT_FLOAT_EQ(result, 6.0F);
}

// ============================================================================
// isPositiveSemiDefinite Tests
// ============================================================================

TEST(DiagonalMatrix, isPositiveSemiDefinite_AllPositive__Success) // NOLINT
{
  // Create a diagonal matrix with all positive elements
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: true (all diagonal elements > 0)
  EXPECT_TRUE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_WithZero__Success) // NOLINT
{
  // Create a diagonal matrix with one zero element
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0, 0},
    {0, 0, 0},
    {0, 0, 3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: true (all diagonal elements >= 0, including zero)
  EXPECT_TRUE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_AllZero__Success) // NOLINT
{
  // Create a zero diagonal matrix
  const auto diagMat = DiagonalMatrix<float32, 3>{};

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: true (all diagonal elements = 0)
  EXPECT_TRUE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_WithNegative__Success) // NOLINT
{
  // Create a diagonal matrix with one negative element
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {1, 0,  0},
    {0, 2,  0},
    {0, 0, -3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: false (one diagonal element < 0)
  EXPECT_FALSE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_MixedSigns__Success) // NOLINT
{
  // Create a diagonal matrix with mixed positive and negative elements
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {-1, 0,  0},
    { 0, 2,  0},
    { 0, 0, -3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: false (contains negative diagonal elements)
  EXPECT_FALSE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_IdentityMatrix__Success) // NOLINT
{
  // Create an identity matrix
  const auto identity = DiagonalMatrix<float32, 3>::Identity();

  // call UUT
  const auto result = identity.isPositiveSemiDefinite();

  // Expected: true (identity matrix has all diagonal elements = 1)
  EXPECT_TRUE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_SmallPositiveValues__Success) // NOLINT
{
  // Create a diagonal matrix with very small positive values
  const auto val = std::numeric_limits<float32>::min();
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 3>({
    {val,   0,   0},
    {  0, val,   0},
    {  0,   0, val}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: true (all diagonal elements > 0, even if very small)
  EXPECT_TRUE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_DoublePrecision__Success) // NOLINT
{
  // Create a diagonal matrix with double precision
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float64, 3>({
    {1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 3.0}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: true (all diagonal elements >= 0)
  EXPECT_TRUE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_LargeMatrix__Success) // NOLINT
{
  // Create a larger diagonal matrix (4x4) with mixed values
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 4>({
    {0, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 2, 0},
    {0, 0, 0, 3}
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: true (all diagonal elements >= 0)
  EXPECT_TRUE(result);
}

TEST(DiagonalMatrix, isPositiveSemiDefinite_EdgeCaseNegativeZero__Success) // NOLINT
{
  // Create a diagonal matrix with negative zero (should be treated as zero)
  // clang-format off
  const auto diagMat = conversions::DiagonalFromList<float32, 2>({
    {0.0f, 0.0f},
    {0.0f, -0.0f}  // negative zero
  });
  // clang-format on

  // call UUT
  const auto result = diagMat.isPositiveSemiDefinite();

  // Expected: true (negative zero should be treated as zero)
  EXPECT_TRUE(result);
}
