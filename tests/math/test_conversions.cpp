#include "gtest/gtest.h"
#include "base/atomic_types.h"
#include "trackingLib/math/linalg/conversions/conversions.h"                     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"          // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"            // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"            // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp"        // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"            // IWYU pragma: keep
#include "trackingLib/math/linalg/covariance_matrix_factored.hpp"                // IWYU pragma: keep
#include "trackingLib/math/linalg/covariance_matrix_full.hpp"                    // IWYU pragma: keep
#include "trackingLib/math/linalg/diagonal_matrix.hpp"                           // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix.hpp"                                    // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.hpp"                             // IWYU pragma: keep
#include "trackingLib/math/linalg/triangular_matrix.hpp"                         // IWYU pragma: keep
#include "trackingLib/math/linalg/vector.hpp"                                    // IWYU pragma: keep

using namespace tracking::math;

namespace
{
/// \brief Helper class to support typed tests which wraps the IsRowMajor param into a type
template <bool IsRowMajor_>
struct MatrixStorageType
{
  static constexpr auto IsRowMajor = IsRowMajor_;
};

/// \brief Helper class for covariance matrix types
template <typename CovarianceType>
struct CovarianceMatrixTypeHelper
{
  using Type = CovarianceType;
};

} // namespace

/// \brief Generic Matrix test class templatized by MatrixStorageType
template <typename MatrixStorageType>
class GTestConversions: public ::testing::Test
{
protected:
  using IntSquareMatType       = SquareMatrix<sint32, 3, MatrixStorageType::IsRowMajor>;
  using FloatSquareMatType     = SquareMatrix<float32, 3, MatrixStorageType::IsRowMajor>;
  using IntDiagMatType         = DiagonalMatrix<sint32, 3>;
  using FloatDiagMatType       = DiagonalMatrix<float32, 3>;
  using IntTriangularUpperType = TriangularMatrix<sint32, 3, false, MatrixStorageType::IsRowMajor>;
  using IntTriangularLowerType = TriangularMatrix<sint32, 3, true, MatrixStorageType::IsRowMajor>;
  using IntVecType             = Vector<sint32, 3>;
  using IntMatType             = Matrix<sint32, 3, 1, true>; // Column matrix for vector conversion

  void SetUp() override
  {
    // clang-format off
    _testSquareMat = IntSquareMatType::FromList({
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9},
    });
    _testDiagMat = IntDiagMatType::FromList({1, 5, 9});
    _testVec = IntVecType::FromList({10, 20, 30});
    // clang-format on
  }

  IntSquareMatType _testSquareMat{};
  IntDiagMatType   _testDiagMat{};
  IntVecType       _testVec{};
};

using ::testing::Types;
// The list of types we want to test.
using MatrixStorageImplementations = Types<MatrixStorageType<true>, MatrixStorageType<false>>;
TYPED_TEST_SUITE(GTestConversions, MatrixStorageImplementations);

// ============================================================================
// Diagonal Conversions
// ============================================================================

TYPED_TEST(GTestConversions, DiagonalFromSquare__Success) // NOLINT
{
  const auto result = conversions::DiagonalFromSquare(this->_testSquareMat);

  EXPECT_EQ(result.at_unsafe(0), 1);
  EXPECT_EQ(result.at_unsafe(1), 5);
  EXPECT_EQ(result.at_unsafe(2), 9);
}

// ============================================================================
// Square Conversions
// ============================================================================

TYPED_TEST(GTestConversions, SquareFromDiagonal__Success) // NOLINT
{
  const auto result = conversions::SquareFromDiagonal<sint32, 3, TypeParam::IsRowMajor>(this->_testDiagMat);

  EXPECT_EQ(result.at_unsafe(0, 0), 1);
  EXPECT_EQ(result.at_unsafe(0, 1), 0);
  EXPECT_EQ(result.at_unsafe(0, 2), 0);
  EXPECT_EQ(result.at_unsafe(1, 0), 0);
  EXPECT_EQ(result.at_unsafe(1, 1), 5);
  EXPECT_EQ(result.at_unsafe(1, 2), 0);
  EXPECT_EQ(result.at_unsafe(2, 0), 0);
  EXPECT_EQ(result.at_unsafe(2, 1), 0);
  EXPECT_EQ(result.at_unsafe(2, 2), 9);
}

TYPED_TEST(GTestConversions, SquareFromList__Success) // NOLINT
{
  // clang-format off
  const auto result = conversions::SquareFromList<sint32, 3, TypeParam::IsRowMajor>({
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

TYPED_TEST(GTestConversions, SquareFromList_InvalidRows__ThrowsRuntimeError) // NOLINT
{
  // clang-format off
  auto throwFunc = []() {
    return conversions::SquareFromList<sint32, 3, TypeParam::IsRowMajor>({
        {1, 2, 3},
        {4, 5, 6},
    });
  };
  // clang-format on
  EXPECT_THROW(throwFunc(), std::runtime_error);
}

TYPED_TEST(GTestConversions, SquareFromList_InvalidCols__ThrowsRuntimeError) // NOLINT
{
  // clang-format off
  auto throwFunc = []() {
    return conversions::SquareFromList<sint32, 3, TypeParam::IsRowMajor>({
        {1, 2},
        {4, 5},
        {7, 8},
    });
  };
  // clang-format on
  EXPECT_THROW(throwFunc(), std::runtime_error);
}

// ============================================================================
// Triangular Conversions
// ============================================================================

TYPED_TEST(GTestConversions, TriangularFromSquare_Upper__Success) // NOLINT
{
  const auto result = conversions::TriangularFromSquare<sint32, 3, false, TypeParam::IsRowMajor>(this->_testSquareMat);

  // Upper triangular: elements on and above diagonal should be preserved
  EXPECT_EQ(result.at_unsafe(0, 0), 1);
  EXPECT_EQ(result.at_unsafe(0, 1), 2);
  EXPECT_EQ(result.at_unsafe(0, 2), 3);
  EXPECT_EQ(result.at_unsafe(1, 1), 5);
  EXPECT_EQ(result.at_unsafe(1, 2), 6);
  EXPECT_EQ(result.at_unsafe(2, 2), 9);
}

TYPED_TEST(GTestConversions, TriangularFromSquare_Lower__Success) // NOLINT
{
  const auto result = conversions::TriangularFromSquare<sint32, 3, true, TypeParam::IsRowMajor>(this->_testSquareMat);

  // Lower triangular: elements on and below diagonal should be preserved
  EXPECT_EQ(result.at_unsafe(0, 0), 1);
  EXPECT_EQ(result.at_unsafe(1, 0), 4);
  EXPECT_EQ(result.at_unsafe(1, 1), 5);
  EXPECT_EQ(result.at_unsafe(2, 0), 7);
  EXPECT_EQ(result.at_unsafe(2, 1), 8);
  EXPECT_EQ(result.at_unsafe(2, 2), 9);
}

TYPED_TEST(GTestConversions, TriangularFromList_Upper__Success) // NOLINT
{
  // clang-format off
  const auto result = conversions::TriangularFromList<sint32, 3, false, TypeParam::IsRowMajor>({
      {1, 2, 3},
      {0, 5, 6},
      {0, 0, 9},
  });
  // clang-format on

  EXPECT_EQ(result.at_unsafe(0, 0), 1);
  EXPECT_EQ(result.at_unsafe(0, 1), 2);
  EXPECT_EQ(result.at_unsafe(0, 2), 3);
  EXPECT_EQ(result.at_unsafe(1, 1), 5);
  EXPECT_EQ(result.at_unsafe(1, 2), 6);
  EXPECT_EQ(result.at_unsafe(2, 2), 9);
}

// ============================================================================
// Vector Conversions
// ============================================================================

TEST(GTestConversionsSpecial, VectorFromList__Success) // NOLINT
{
  const auto result = conversions::VectorFromList<sint32, 3>({1, 2, 3});

  EXPECT_EQ(result.at_unsafe(0), 1);
  EXPECT_EQ(result.at_unsafe(1), 2);
  EXPECT_EQ(result.at_unsafe(2), 3);
}

TEST(GTestConversionsSpecial, VectorFromMatrixColumnView__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::MatrixFromList<sint32, 3, 1, true>({
      {10},
      {20},
      {30},
  });
  // clang-format on
  const auto colView = MatrixColumnView<sint32, 3, 1, true>{mat, 0, 0, 2};
  const auto result  = conversions::VectorFromMatrixColumnView<sint32, 3>(colView);

  EXPECT_EQ(result.at_unsafe(0), 10);
  EXPECT_EQ(result.at_unsafe(1), 20);
  EXPECT_EQ(result.at_unsafe(2), 30);
}

// ============================================================================
// Matrix Conversions
// ============================================================================

TYPED_TEST(GTestConversions, MatrixFromList__Success) // NOLINT
{
  // clang-format off
  const auto result = 
    conversions::MatrixFromList<sint32, 2, 3, TypeParam::IsRowMajor>({
      {1, 2, 3},
      {4, 5, 6},
    });
  
  auto resultExp = std::vector<sint32>{1, 2, 3, 4, 5, 6};
  // clang-format on

  size_t index = 0;
  for (sint32 r = 0; r < result.Rows; ++r)
  {
    for (sint32 c = 0; c < result.Cols; ++c)
    {
      EXPECT_EQ(result.at_unsafe(r, c), resultExp[index]);
      ++index;
    }
  }
}

TYPED_TEST(GTestConversions, MatrixFromList_InvalidRows__ThrowsRuntimeError) // NOLINT
{
  // clang-format off
  auto throwFunc = []() {
    return conversions::MatrixFromList<sint32, 2, 3, TypeParam::IsRowMajor>({
        {1, 2, 3},
    });
  };
  // clang-format on
  EXPECT_THROW(throwFunc(), std::runtime_error);
}

TYPED_TEST(GTestConversions, MatrixFromList_InvalidCols__ThrowsRuntimeError) // NOLINT
{
  // clang-format off
  auto throwFunc = []() {
    return conversions::MatrixFromList<sint32, 2, 3, TypeParam::IsRowMajor>({
        {1, 2},
        {4, 5},
    });
  };
  // clang-format on
  EXPECT_THROW(throwFunc(), std::runtime_error);
}

TEST(GTestConversionsSpecial, MatrixFromVector__Success) // NOLINT
{
  const auto vec    = conversions::VectorFromList<sint32, 3>({1, 2, 3});
  const auto result = conversions::MatrixFromVector(vec);

  EXPECT_EQ(result.at_unsafe(0, 0), 1);
  EXPECT_EQ(result.at_unsafe(1, 0), 2);
  EXPECT_EQ(result.at_unsafe(2, 0), 3);
}

// ============================================================================
// Covariance Matrix Conversions
// ============================================================================

TEST(GTestConversionsSpecial, CovarianceMatrixFullFromList__Success) // NOLINT
{
  // clang-format off
  const auto result = CovarianceMatrixFull<float32,3>::FromList({
      {1.5F, 1.0F, 0.0F},
      {1.0F, 2.0F, 0.0F},
      {0.0F, 0.0F, 3.0F},
  });
  // clang-format on

  EXPECT_FLOAT_EQ(result.at_unsafe(0, 0), 1.5F);
  EXPECT_FLOAT_EQ(result.at_unsafe(0, 1), 1.0F);
  EXPECT_FLOAT_EQ(result.at_unsafe(1, 1), 2.0F);
  EXPECT_FLOAT_EQ(result.at_unsafe(2, 2), 3.0F);
}

TEST(GTestConversionsSpecial, CovarianceMatrixFactoredFromList_SingleList__Success) // NOLINT
{
  // clang-format off
  const auto result = conversions::CovarianceMatrixFactoredFromList<float32,3>({
      {1.5F, 1.0F, 0.0F},
      {1.0F, 2.0F, 0.0F},
      {0.0F, 0.0F, 3.0F},
  });
  // clang-format on

  EXPECT_FLOAT_EQ(result.at_unsafe(0, 0), 1.5F);
  EXPECT_FLOAT_EQ(result.at_unsafe(0, 1), 1.0F);
  EXPECT_FLOAT_EQ(result.at_unsafe(1, 1), 2.0F);
  EXPECT_FLOAT_EQ(result.at_unsafe(2, 2), 3.0F);
}

TEST(GTestConversionsSpecial, CovarianceMatrixFactoredFromList_SeparateLists__Success) // NOLINT
{
  // clang-format off
  const auto result = CovarianceMatrixFactored<float32, 3>::FromList({
      {1.0F, 0.5F, 0.0F},
      {0.0F, 1.0F, 0.0F},
      {0.0F, 0.0F, 1.0F},
  }, {1.0F, 2.0F, 3.0F});
  // clang-format on

  EXPECT_FLOAT_EQ(result.at_unsafe(0, 0), 1.5F);
  EXPECT_FLOAT_EQ(result.at_unsafe(0, 1), 1.0F);
  EXPECT_FLOAT_EQ(result.at_unsafe(1, 1), 2.0F);
  EXPECT_FLOAT_EQ(result.at_unsafe(2, 2), 3.0F);
}
