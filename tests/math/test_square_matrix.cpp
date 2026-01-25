#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.hpp"                  // IWYU pragma: keep

using namespace tracking::math;

TEST(SquareMatrix, householderQR) // NOLINT
{
  // Create a square matrix for testing
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  // clang-format on

  // call UUT
  const auto [Q, R] = mat.householderQR();

  // Check if QR is a valid decomposition by comparing to the original matrix
  const auto recomposed = Q * R;
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_FLOAT_EQ(mat.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrix, inverse) // NOLINT
{
  // Create a square matrix for testing
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    { 9.25, -6.0,  1.25},
    {-6.00,  4.5, -1.00},
    { 1.25, -1.0,  0.25}
  });
  const auto expInvMat = SquareMatrix<float32, 3, true>::FromList({
    {1.0,  2.0,  3.0},
    {2.0,  6.0, 14.0},
    {3.0, 14.0, 45.0}
  });
  // clang-format on

  // call UUT
  const auto inv = mat.inverse();

  // compare to expected inverse
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_NEAR(inv.at_unsafe(row, col), expInvMat.at_unsafe(row, col), 7e-5);
    }
  }
}

TEST(SquareMatrix, decomposeLLT) // NOLINT
{
  // clang-format off
  auto cov = SquareMatrix<float32, 6, true>::FromList({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_TRUE(retVal.has_value());
  const auto& L = retVal.value();

  const auto recomposed = L * L.transpose();
  for (auto row = 0; row < 6; row++)
  {
    for (auto col = 0; col < 6; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrix, decomposeLLT_NotSymmetric_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, -4},
    {-3, 13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrix, decomposeLLT_SymmetricNotPositiveDefinite_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, -3},
    {-3, -13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLLT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrix, symmetrize) // NOLINT
{
  // clang-format off
  auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  });
  // clang-format on

  // call UUT
  mat.symmetrize();

  EXPECT_TRUE(mat.isSymmetric());

  // Check values: (A + A^T) / 2
  // (2+4)/2 = 3
  // (3+7)/2 = 5
  // (6+8)/2 = 7
  EXPECT_FLOAT_EQ(mat.at_unsafe(0, 1), 3.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(1, 0), 3.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(0, 2), 5.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(2, 0), 5.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(1, 2), 7.0F);
  EXPECT_FLOAT_EQ(mat.at_unsafe(2, 1), 7.0F);
}

TEST(SquareMatrix, decomposeUDUT) // NOLINT
{
  // clang-format off
  auto cov = SquareMatrix<float32, 3, true>::FromList({
    {10, 2, 1},
    { 2, 5, 1},
    { 1, 1, 2}
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_TRUE(retVal.has_value());
  const auto [U, D] = retVal.value();

  // Verify U is unit upper triangular
  EXPECT_TRUE(U.isUnitUpperTriangular());

  // Verify reconstruction P = U * D * U^T
  const auto recomposed = (U * D) * U.transpose();
  for (auto row = 0; row < 3; row++)
  {
    for (auto col = 0; col < 3; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrix, decomposeUDUT_NotSymmetric_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, 2},
    { 1, 5}
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeUDUT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrix, decomposeLDLT) // NOLINT
{
  // clang-format off
  auto cov = SquareMatrix<float32, 6, true>::FromList({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_TRUE(retVal.has_value());
  const auto [L, D]     = retVal.value();
  const auto recomposed = (L * D) * L.transpose();
  for (auto row = 0; row < 6; row++)
  {
    for (auto col = 0; col < 6; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), recomposed.at_unsafe(row, col));
    }
  }
}

TEST(SquareMatrix, decomposeLDLT_NotSymmetric_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, -4},
    {-3, 13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_FALSE(retVal.has_value());
}

TEST(SquareMatrix, decomposeLDLT_SymmetricNotPositiveDefinite_ExpectError) // NOLINT
{
  // clang-format off
  auto cov = SquareMatrix<float32, 2, true>::FromList({
    {10, -3},
    {-3, -13},
  });
  // clang-format on

  // call UUT
  auto retVal = cov.decomposeLDLT();

  EXPECT_FALSE(retVal.has_value());
}

// ============================================================================
// Matrix FromList ctor tests
// ============================================================================

/// \brief Helper class to support typed tests which wraps the IsRowMajor param into a type
template <bool IsRowMajor_>
struct MatrixStorageType
{
  static constexpr auto IsRowMajor = IsRowMajor_;
};

/// \brief Generic Matrix test class templatized by MatrixStorageType
template <typename MatrixStorageType>
class GTestSquareMatrix: public ::testing::Test
{
};

using ::testing::Types;
// The list of types we want to test.
using MatrixStorageImplementations = Types<MatrixStorageType<true>, MatrixStorageType<false>>;
TYPED_TEST_SUITE(GTestSquareMatrix, MatrixStorageImplementations);

TYPED_TEST(GTestSquareMatrix, ctor_FromList__Success) // NOLINT
{
  // clang-format off
  // call UUT
  const auto result = SquareMatrix<sint32, 3, TypeParam::IsRowMajor>::FromList({
      {1, 2, 3},
      {4, 5, 6},
      {7, 8, 9},
  });

  auto resultExp = std::vector<sint32>{1, 2, 3, 4, 5, 6, 7, 8, 9};
  // clang-format on

  size_t index = 0;
  for (sint32 r = 0; r < 3; ++r)
  {
    for (sint32 c = 0; c < 3; ++c)
    {
      EXPECT_EQ(result.at_unsafe(r, c), resultExp[index]);
      ++index;
    }
  }
}

TYPED_TEST(GTestSquareMatrix, ctor_FromList_InvalidRows__ThrowsRuntimeError) // NOLINT
{
  // clang-format off
  auto throwFunc = []() {
    // call UUT
    return SquareMatrix<sint32, 3, TypeParam::IsRowMajor>::FromList({
        {1, 2, 3},
        {4, 5, 6},
    });
  };
  // clang-format on
  EXPECT_THROW(throwFunc(), std::runtime_error);
}

TYPED_TEST(GTestSquareMatrix, ctor_FromList_InvalidCols__ThrowsRuntimeError) // NOLINT
{
  // clang-format off
  auto throwFunc = []() {
    // call UUT
    return SquareMatrix<sint32, 3, TypeParam::IsRowMajor>::FromList({
        {1, 2},
        {4, 5},
        {7, 8},
    });
  };
  // clang-format on
  EXPECT_THROW(throwFunc(), std::runtime_error);
}

// ============================================================================
// Matrix Property Check Tests
// ============================================================================

TEST(SquareMatrix, isOrthogonal_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.isOrthogonal();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isOrthogonal_NonOrthogonalMatrix__Fail) // NOLINT
{
  // Create a non-orthogonal matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 2, true>::FromList({
    {1, 2},
    {3, 4}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isOrthogonal();

  EXPECT_FALSE(result);
}

TEST(SquareMatrix, isOrthogonal_OrthogonalMatrix__Success) // NOLINT
{
  // Create an orthogonal matrix (rotation matrix)
  const float32 cos_theta = std::cos(0.5F);
  const float32 sin_theta = std::sin(0.5F);

  // clang-format off
  const auto mat = SquareMatrix<float32, 2, true>::FromList({
    {cos_theta, -sin_theta},
    {sin_theta, cos_theta}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isOrthogonal(1e-5F);

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isUpperTriangular_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.isUpperTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isUpperTriangular_UpperTriangularMatrix__Success) // NOLINT
{
  // Create an upper triangular matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},
    {0, 4, 5},
    {0, 0, 6}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isUpperTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isUpperTriangular_NonUpperTriangularMatrix__Fail) // NOLINT
{
  // Create a non-upper triangular matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},
    {4, 5, 6},  // Has non-zero element below diagonal
    {7, 8, 9}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isUpperTriangular();

  EXPECT_FALSE(result);
}

TEST(SquareMatrix, isLowerTriangular_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.isLowerTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isLowerTriangular_LowerTriangularMatrix__Success) // NOLINT
{
  // Create a lower triangular matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 0, 0},
    {2, 3, 0},
    {4, 5, 6}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isLowerTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isLowerTriangular_NonLowerTriangularMatrix__Fail) // NOLINT
{
  // Create a non-lower triangular matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},  // Has non-zero elements above diagonal
    {4, 5, 6},
    {7, 8, 9}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isLowerTriangular();

  EXPECT_FALSE(result);
}

TEST(SquareMatrix, hasUnitDiagonal_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.hasUnitDiagonal();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, hasUnitDiagonal_UnitDiagonalMatrix__Success) // NOLINT
{
  // Create a matrix with unit diagonal
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},
    {4, 1, 6},
    {7, 8, 1}
  });
  // clang-format on

  // call UUT
  const auto result = mat.hasUnitDiagonal();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, hasUnitDiagonal_NonUnitDiagonalMatrix__Fail) // NOLINT
{
  // Create a matrix without unit diagonal
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {2, 2, 3},  // Diagonal element is 2, not 1
    {4, 1, 6},
    {7, 8, 1}
  });
  // clang-format on

  // call UUT
  const auto result = mat.hasUnitDiagonal();

  EXPECT_FALSE(result);
}

TEST(SquareMatrix, isOrthogonal_WithTolerance__Success) // NOLINT
{
  // Create a matrix that's almost orthogonal but not exactly
  // clang-format off
  const auto mat = SquareMatrix<float32, 2, true>::FromList({
    {0.999F, -0.01F},
    {0.01F, 0.999F}
  });
  // clang-format on

  // Should pass with loose tolerance
  EXPECT_TRUE(mat.isOrthogonal(0.02F));

  // Should fail with strict tolerance
  EXPECT_FALSE(mat.isOrthogonal(0.001F));
}

TEST(SquareMatrix, isUpperTriangular_WithTolerance__Success) // NOLINT
{
  // Create a matrix that's almost upper triangular
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},
    {0.001F, 4, 5},  // Small non-zero element below diagonal
    {0, 0, 6}
  });
  // clang-format on

  // Should pass with loose tolerance
  EXPECT_TRUE(mat.isUpperTriangular(0.01F));

  // Should fail with strict tolerance
  EXPECT_FALSE(mat.isUpperTriangular(0.0001F));
}

TEST(SquareMatrix, isOrthogonal_Double__Success) // NOLINT
{
  const auto identity = SquareMatrix<float64, 3, true>::Identity();

  // call UUT
  const auto result = identity.isOrthogonal();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, isUpperTriangular_Double__Success) // NOLINT
{
  // Create an upper triangular matrix with double precision
  // clang-format off
  const auto mat = SquareMatrix<float64, 3, true>::FromList({
    {1.0, 2.0, 3.0},
    {0.0, 4.0, 5.0},
    {0.0, 0.0, 6.0}
  });
  // clang-format on

  // call UUT
  const auto result = mat.isUpperTriangular();

  EXPECT_TRUE(result);
}

TEST(SquareMatrix, hasUnitDiagonal_Double__Success) // NOLINT
{
  const auto identity = SquareMatrix<float64, 3, true>::Identity();

  // call UUT
  const auto result = identity.hasUnitDiagonal();

  EXPECT_TRUE(result);
}

// ============================================================================
// Trace and Determinant Tests
// ============================================================================

TEST(SquareMatrix, trace_2x2__Success) // NOLINT
{
  // Create a 2x2 matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 2, true>::FromList({
    {1, 2},
    {3, 4}
  });
  // clang-format on

  // call UUT
  const auto result = mat.trace();

  // Expected: 1 + 4 = 5
  EXPECT_FLOAT_EQ(result, 5.0F);
}

TEST(SquareMatrix, trace_3x3__Success) // NOLINT
{
  // Create a 3x3 matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  });
  // clang-format on

  // call UUT
  const auto result = mat.trace();

  // Expected: 1 + 5 + 9 = 15
  EXPECT_FLOAT_EQ(result, 15.0F);
}

TEST(SquareMatrix, trace_4x4__Success) // NOLINT
{
  // Create a 4x4 matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 4, true>::FromList({
    {1,  2,  3,  4},
    {5,  6,  7,  8},
    {9, 10, 11, 12},
    {13, 14, 15, 16}
  });
  // clang-format on

  // call UUT
  const auto result = mat.trace();

  // Expected: 1 + 6 + 11 + 16 = 34
  EXPECT_FLOAT_EQ(result, 34.0F);
}

TEST(SquareMatrix, trace_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.trace();

  // Expected: 1 + 1 + 1 = 3 (matrix size)
  EXPECT_FLOAT_EQ(result, 3.0F);
}

TEST(SquareMatrix, trace_ZeroMatrix__Success) // NOLINT
{
  const auto zeroMat = SquareMatrix<float32, 3, true>{};

  // call UUT
  const auto result = zeroMat.trace();

  // Expected: 0 + 0 + 0 = 0
  EXPECT_FLOAT_EQ(result, 0.0F);
}

TEST(SquareMatrix, trace_Double__Success) // NOLINT
{
  // Create a 3x3 matrix with double precision
  // clang-format off
  const auto mat = SquareMatrix<float64, 3, true>::FromList({
    {1.0, 2.0, 3.0},
    {4.0, 5.0, 6.0},
    {7.0, 8.0, 9.0}
  });
  // clang-format on

  // call UUT
  const auto result = mat.trace();

  // Expected: 1.0 + 5.0 + 9.0 = 15.0
  EXPECT_DOUBLE_EQ(result, 15.0);
}

TEST(SquareMatrix, determinant_2x2__Success) // NOLINT
{
  // Create a 2x2 matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 2, true>::FromList({
    {1, 2},
    {3, 4}
  });
  // clang-format on

  // call UUT
  const auto result = mat.determinant();

  // Expected: (1*4) - (2*3) = 4 - 6 = -2
  EXPECT_FLOAT_EQ(result, -2.0F);
}

TEST(SquareMatrix, determinant_3x3__Success) // NOLINT
{
  // Create a 3x3 matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},
    {0, 1, 4},
    {5, 6, 0}
  });
  // clang-format on

  // call UUT
  const auto result = mat.determinant();

  // Expected: 1*(1*0 - 4*6) - 2*(0*0 - 4*5) + 3*(0*6 - 1*5)
  //          = 1*(-24) - 2*(-20) + 3*(-5)
  //          = -24 + 40 - 15 = 1
  EXPECT_NEAR(result, 1.0F, 1e-5F);
}

TEST(SquareMatrix, determinant_IdentityMatrix__Success) // NOLINT
{
  const auto identity = SquareMatrix<float32, 3, true>::Identity();

  // call UUT
  const auto result = identity.determinant();

  // Expected: 1 (identity matrix determinant is always 1)
  EXPECT_FLOAT_EQ(result, 1.0F);
}

TEST(SquareMatrix, determinant_SingularMatrix__Success) // NOLINT
{
  // Create a singular matrix (determinant should be 0)
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1, 2, 3},
    {1, 2, 3},
    {4, 5, 6}
  });
  // clang-format on

  // call UUT
  const auto result = mat.determinant();

  // Expected: 0 (rows 1 and 2 are identical, matrix is singular)
  EXPECT_NEAR(result, 0.0F, 1e-5F);
}

TEST(SquareMatrix, determinant_IllConditionedMatrix__Success) // NOLINT
{
  // Create an ill-conditioned matrix
  // clang-format off
  const auto mat = SquareMatrix<float32, 3, true>::FromList({
    {1.0, 2.0, 3.0},
    {2.1, 4.2, 6.3},
    {3.0, 6.0, 9.0}
  });
  // clang-format on

  // call UUT
  const auto result = mat.determinant();

  // Expected: very small value (matrix is nearly singular)
  EXPECT_NEAR(result, 0.0F, 1e-3F);
}

TEST(SquareMatrix, determinant_Double__Success) // NOLINT
{
  // Create a 3x3 matrix with double precision
  // clang-format off
  const auto mat = SquareMatrix<float64, 3, true>::FromList({
    {1.0, 2.0, 3.0},
    {0.0, 1.0, 4.0},
    {5.0, 6.0, 0.0}
  });
  // clang-format on

  // call UUT
  const auto result = mat.determinant();

  // Expected: 1.0
  EXPECT_NEAR(result, 1.0, 1e-10);
}