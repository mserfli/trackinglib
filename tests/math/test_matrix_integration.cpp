#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/conversions.h"              // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"   // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/diagonal_matrix.hpp"                    // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.hpp"                      // IWYU pragma: keep
#include "trackingLib/math/linalg/triangular_matrix.hpp"                  // IWYU pragma: keep

using namespace tracking::math;

// ============================================================================
// Integration Tests for Trace and Determinant across Matrix Types
// ============================================================================

TEST(MatrixIntegration, Trace_Consistency_DiagonalVsSquare__Success) // NOLINT
{
  // Create a diagonal matrix
  // clang-format off
  const auto diagMat = DiagonalMatrix<float32, 3>::FromList({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  // clang-format on

  // Convert to square matrix
  const auto squareMat = SquareMatrix<float32, 3, true>{diagMat};

  // call UUT
  const auto diagTrace   = diagMat.trace();
  const auto squareTrace = squareMat.trace();

  // Both should have the same trace
  EXPECT_FLOAT_EQ(diagTrace, squareTrace);
  EXPECT_FLOAT_EQ(diagTrace, 6.0F); // 1 + 2 + 3
}

TEST(MatrixIntegration, Trace_Consistency_DiagonalVsTriangular__Success) // NOLINT
{
  // Create a diagonal matrix
  // clang-format off
  const auto diagMat = DiagonalMatrix<float32, 3>::FromList({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  // clang-format on

  // Convert to upper triangular matrix
  const auto triuMat = TriangularMatrix<float32, 3, false, true>::FromList({{1, 0, 0}, {0, 2, 0}, {0, 0, 3}});

  // call UUT
  const auto diagTrace = diagMat.trace();
  const auto triuTrace = triuMat.trace();

  // Both should have the same trace
  EXPECT_FLOAT_EQ(diagTrace, triuTrace);
  EXPECT_FLOAT_EQ(diagTrace, 6.0F); // 1 + 2 + 3
}

TEST(MatrixIntegration, Determinant_Consistency_DiagonalVsSquare__Success) // NOLINT
{
  // Create a diagonal matrix
  // clang-format off
  const auto diagMat = DiagonalMatrix<float32, 3>::FromList({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  // clang-format on

  // Convert to square matrix
  const auto squareMat = SquareMatrix<float32, 3, true>{diagMat};

  // call UUT
  const auto diagDet   = diagMat.determinant();
  const auto squareDet = squareMat.determinant();

  // Both should have the same determinant
  EXPECT_FLOAT_EQ(diagDet, squareDet);
  EXPECT_FLOAT_EQ(diagDet, 6.0F); // 1 * 2 * 3
}

TEST(MatrixIntegration, Determinant_Consistency_DiagonalVsTriangular__Success) // NOLINT
{
  // Create a diagonal matrix
  // clang-format off
  const auto diagMat = DiagonalMatrix<float32, 3>::FromList({
    {1, 0, 0},
    {0, 2, 0},
    {0, 0, 3}
  });
  // clang-format on

  // Convert to upper triangular matrix
  const auto triuMat = TriangularMatrix<float32, 3, false, true>::FromList({{1, 0, 0}, {0, 2, 0}, {0, 0, 3}});

  // call UUT
  const auto diagDet = diagMat.determinant();
  const auto triuDet = triuMat.determinant();

  // Both should have the same determinant
  EXPECT_FLOAT_EQ(diagDet, triuDet);
  EXPECT_FLOAT_EQ(diagDet, 6.0F); // 1 * 2 * 3
}

TEST(MatrixIntegration, Determinant_Consistency_UpperVsLowerTriangular__Success) // NOLINT
{
  // Create upper and lower triangular matrices with same diagonal
  // clang-format off
  const auto triuMat = TriangularMatrix<float32, 3, false, true>::FromList({
    {1, 2, 3},
    {0, 4, 5},
    {0, 0, 6}
  });
  
  const auto trilMat = TriangularMatrix<float32, 3, true, true>::FromList({
    {1, 0, 0},
    {2, 4, 0},
    {3, 5, 6}
  });
  // clang-format on

  // call UUT
  const auto triuDet = triuMat.determinant();
  const auto trilDet = trilMat.determinant();

  // Both should have the same determinant (product of diagonal elements)
  EXPECT_FLOAT_EQ(triuDet, trilDet);
  EXPECT_FLOAT_EQ(triuDet, 24.0F); // 1 * 4 * 6
}

TEST(MatrixIntegration, Trace_DoublePrecision_Consistency__Success) // NOLINT
{
  // Create diagonal matrix with double precision
  // clang-format off
  const auto diagMat = DiagonalMatrix<float64, 3>::FromList({
    {1.0, 0.0, 0.0},
    {0.0, 2.0, 0.0},
    {0.0, 0.0, 3.0}
  });
  // clang-format on

  // Convert to square matrix
  const auto squareMat = SquareMatrix<float64, 3, true>{diagMat};

  // call UUT
  const auto diagTrace   = diagMat.trace();
  const auto squareTrace = squareMat.trace();

  // Both should have the same trace
  EXPECT_DOUBLE_EQ(diagTrace, squareTrace);
  EXPECT_DOUBLE_EQ(diagTrace, 6.0); // 1.0 + 2.0 + 3.0
}

TEST(MatrixIntegration, Determinant_DoublePrecision_Consistency__Success) // NOLINT
{
  // Create diagonal matrix with double precision
  // clang-format off
  const auto diagMat = DiagonalMatrix<float64, 3>::FromList({
    {1.0, 0.0, 0.0},
    {0.0, 2.0, 0.0},
    {0.0, 0.0, 3.0}
  });
  // clang-format on

  // Convert to square matrix
  const auto squareMat = SquareMatrix<float64, 3, true>{diagMat};

  // call UUT
  const auto diagDet   = diagMat.determinant();
  const auto squareDet = squareMat.determinant();

  // Both should have the same determinant
  EXPECT_DOUBLE_EQ(diagDet, squareDet);
  EXPECT_DOUBLE_EQ(diagDet, 6.0); // 1.0 * 2.0 * 3.0
}

TEST(MatrixIntegration, Trace_IdentityMatrix_AllTypes__Success) // NOLINT
{
  // Test identity matrices of different types
  const auto diagIdentity   = DiagonalMatrix<float32, 3>::Identity();
  const auto squareIdentity = SquareMatrix<float32, 3, true>::Identity();
  const auto triuIdentity   = TriangularMatrix<float32, 3, false, true>::Identity();
  const auto trilIdentity   = TriangularMatrix<float32, 3, true, true>::Identity();

  // call UUT
  const auto diagTrace   = diagIdentity.trace();
  const auto squareTrace = squareIdentity.trace();
  const auto triuTrace   = triuIdentity.trace();
  const auto trilTrace   = trilIdentity.trace();

  // All should have the same trace (3 for 3x3 identity)
  EXPECT_FLOAT_EQ(diagTrace, 3.0F);
  EXPECT_FLOAT_EQ(squareTrace, 3.0F);
  EXPECT_FLOAT_EQ(triuTrace, 3.0F);
  EXPECT_FLOAT_EQ(trilTrace, 3.0F);
}

TEST(MatrixIntegration, Determinant_IdentityMatrix_AllTypes__Success) // NOLINT
{
  // Test identity matrices of different types
  const auto diagIdentity   = DiagonalMatrix<float32, 3>::Identity();
  const auto squareIdentity = SquareMatrix<float32, 3, true>::Identity();
  const auto triuIdentity   = TriangularMatrix<float32, 3, false, true>::Identity();
  const auto trilIdentity   = TriangularMatrix<float32, 3, true, true>::Identity();

  // call UUT
  const auto diagDet   = diagIdentity.determinant();
  const auto squareDet = squareIdentity.determinant();
  const auto triuDet   = triuIdentity.determinant();
  const auto trilDet   = trilIdentity.determinant();

  // All should have the same determinant (1 for identity matrices)
  EXPECT_FLOAT_EQ(diagDet, 1.0F);
  EXPECT_FLOAT_EQ(squareDet, 1.0F);
  EXPECT_FLOAT_EQ(triuDet, 1.0F);
  EXPECT_FLOAT_EQ(trilDet, 1.0F);
}

TEST(MatrixIntegration, Trace_SingularMatrix_AllTypes__Success) // NOLINT
{
  // Test singular matrices (with zero diagonal elements)
  // clang-format off
  const auto diagMat = DiagonalMatrix<float32, 3>::FromList({
    {1, 0, 0},
    {0, 0, 0},
    {0, 0, 3}
  });
  
  const auto squareMat = SquareMatrix<float32, 3, true>{diagMat};
  const auto triuMat = TriangularMatrix<float32, 3, false, true>::FromList({
    {1, 2, 3},
    {0, 0, 4},
    {0, 0, 5}
  });
  const auto trilMat = TriangularMatrix<float32, 3, true, true>::FromList({
    {1, 0, 0},
    {2, 0, 0},
    {3, 4, 5}
  });
  // clang-format on

  // call UUT
  const auto diagTrace   = diagMat.trace();
  const auto squareTrace = squareMat.trace();
  const auto triuTrace   = triuMat.trace();
  const auto trilTrace   = trilMat.trace();

  // All should have the same trace (1 + 0 + 5 = 6)
  EXPECT_FLOAT_EQ(diagTrace, 4.0F); // 1 + 0 + 3
  EXPECT_FLOAT_EQ(squareTrace, 4.0F);
  EXPECT_FLOAT_EQ(triuTrace, 6.0F); // 1 + 0 + 5
  EXPECT_FLOAT_EQ(trilTrace, 6.0F); // 1 + 0 + 5
}

TEST(MatrixIntegration, Determinant_SingularMatrix_AllTypes__Success) // NOLINT
{
  // Test singular matrices (with zero diagonal elements)
  // clang-format off
  const auto diagMat = DiagonalMatrix<float32, 3>::FromList({
    {1, 0, 0},
    {0, 0, 0},
    {0, 0, 3}
  });
  
  const auto squareMat = SquareMatrix<float32, 3, true>{diagMat};
  const auto triuMat = TriangularMatrix<float32, 3, false, true>::FromList({
    {1, 2, 3},
    {0, 0, 4},
    {0, 0, 5}
  });
  const auto trilMat = TriangularMatrix<float32, 3, true, true>::FromList({
    {1, 0, 0},
    {2, 0, 0},
    {3, 4, 5}
  });
  // clang-format on

  // call UUT
  const auto diagDet   = diagMat.determinant();
  const auto squareDet = squareMat.determinant();
  const auto triuDet   = triuMat.determinant();
  const auto trilDet   = trilMat.determinant();

  // All should have determinant 0 (singular matrices)
  EXPECT_FLOAT_EQ(diagDet, 0.0F);
  EXPECT_FLOAT_EQ(squareDet, 0.0F);
  EXPECT_FLOAT_EQ(triuDet, 0.0F);
  EXPECT_FLOAT_EQ(trilDet, 0.0F);
}