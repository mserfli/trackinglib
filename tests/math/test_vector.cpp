#include "gtest/gtest.h"
#include "base/atomic_types.h"
#include "math/linalg/errors.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"   // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix.hpp"                             // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_column_view.hpp"                 // IWYU pragma: keep
#include "trackingLib/math/linalg/vector.hpp"                             // IWYU pragma: keep

using tracking::math::Errors;
using tracking::math::Matrix;
using tracking::math::Vector;

// Helper struct for typed tests with different value types
template <typename ValueType_>
struct VectorValueType
{
  using type = ValueType_;
};

using VectorTestTypes = testing::Types<VectorValueType<sint32>, VectorValueType<float32>, VectorValueType<float64>>;

TEST(Vector, ctor_FromList__Success) // NOLINT
{
  // call UUT
  auto resultResult = Vector<sint32, 3>::FromList({1, 2, 3});
  ASSERT_TRUE(resultResult.has_value());
  const auto result = resultResult.value();

  EXPECT_EQ(result.at_unsafe(0), 1);
  EXPECT_EQ(result.at_unsafe(1), 2);
  EXPECT_EQ(result.at_unsafe(2), 3);
}


TEST(Vector, UnitVector__Success) // NOLINT
{
  // clang-format off
  auto aResult = Vector<sint32, 4>::FromList({
    0, 0, 1, 0
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  // clang-format on

  // call UUT
  const auto res = Vector<sint32, 4>::UnitVector<2>();

  EXPECT_EQ(a._data, res._data);
}

TEST(Vector, test_op_at_Success) // NOLINT
{
  // clang-format off
  using VecType = Vector<sint32, 4>;
  auto aResult = Vector<sint32, 4>::FromList({
    1, 2, 3, 4
  });
  ASSERT_TRUE(aResult.has_value());
  auto a = aResult.value();
  // clang-format on

  // call UUT
  auto retVal = a[VecType::Rows - 1];

  EXPECT_TRUE(retVal.has_value());
  EXPECT_EQ(retVal.value(), 4);
}

TEST(Vector, test_op_at_const_Success) // NOLINT
{
  // clang-format off
  using VecType = Vector<sint32, 4>;
  auto aResult = Vector<sint32, 4>::FromList({
    1, 2, 3, 4
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  // clang-format on

  // call UUT
  auto retVal = a[VecType::Rows - 1];

  EXPECT_TRUE(retVal.has_value());
  EXPECT_EQ(retVal.value(), 4);
}

TEST(Vector, op_dot__Success) // NOLINT
{
  // clang-format off
  using VecType = Vector<sint32, 4>;
  const auto a  = VecType::Ones();
  auto bResult = Vector<sint32, 4>::FromList({
    1, 2, 3, 4
  });
  ASSERT_TRUE(bResult.has_value());
  const auto b = bResult.value();
  // clang-format on

  // call UUT
  const auto res = a * b;

  EXPECT_EQ(res, 10);
}

TEST(Vector, op_normSq__Success) // NOLINT
{
  // clang-format off
  auto aResult = Vector<sint32, 4>::FromList({
    1, 2, 3, 4
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  // clang-format on

  // call UUT
  const auto res = a.normSq();

  EXPECT_EQ(res, 30);
}

TEST(Vector, op_norm__Success) // NOLINT
{
  // clang-format off
  auto aResult = Vector<float32, 2>::FromList({
    3, 4
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  // clang-format on

  // call UUT
  const auto res = a.norm();

  EXPECT_FLOAT_EQ(res, 5.0F);
}

TEST(Vector, op_normalize__Success) // NOLINT
{
  // clang-format off
  auto aResult = Vector<float32, 2>::FromList({
    3, 4
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  auto bResult = Vector<float32, 2>::FromList({
    3 / 5.0, 4 / 5.0
  });
  ASSERT_TRUE(bResult.has_value());
  const auto b = bResult.value();
  // clang-format on

  // call UUT
  const auto res = a.normalize();

  EXPECT_EQ(res._data, b._data);
}

// Typed tests for vector arithmetic operations
template <typename TypeParam>
class GTestVectorArithmetic: public testing::Test
{
protected:
  using value_type = typename TypeParam::type;
};

TYPED_TEST_SUITE(GTestVectorArithmetic, VectorTestTypes);

TYPED_TEST(GTestVectorArithmetic, op_plus__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto aResult = Vector<value_type, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  auto bResult = Vector<value_type, 3>::FromList({
    4, 5, 6
  });
  ASSERT_TRUE(bResult.has_value());
  const auto b = bResult.value();
  auto expectedResult = Vector<value_type, 3>::FromList({
    5, 7, 9
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT
  const auto res = a + b;

  EXPECT_EQ(res._data, expected._data);
}

TYPED_TEST(GTestVectorArithmetic, op_minus__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto aResult = Vector<value_type, 3>::FromList({
    5, 7, 9
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  auto bResult = Vector<value_type, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(bResult.has_value());
  const auto b = bResult.value();
  auto expectedResult = Vector<value_type, 3>::FromList({
    4, 5, 6
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT
  const auto res = a - b;

  EXPECT_EQ(res._data, expected._data);
}

TYPED_TEST(GTestVectorArithmetic, op_mul_scalar__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto aResult = Vector<value_type, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  const value_type scalar = 2;
  auto expectedResult = Vector<value_type, 3>::FromList({
    2, 4, 6
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT - use scalar * vector (inherited from Matrix)
  const auto res = scalar * a;

  EXPECT_EQ(res._data, expected._data);
}

TYPED_TEST(GTestVectorArithmetic, op_plus_scalar__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto aResult = Vector<value_type, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  const value_type scalar = 10;
  auto expectedResult = Vector<value_type, 3>::FromList({
    11, 12, 13
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT
  const auto res = a + scalar;

  EXPECT_EQ(res._data, expected._data);
}

TYPED_TEST(GTestVectorArithmetic, op_minus_scalar__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto aResult = Vector<value_type, 3>::FromList({
    10, 20, 30
  });
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();
  const value_type scalar = 5;
  auto expectedResult = Vector<value_type, 3>::FromList({
    5, 15, 25
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT
  const auto res = a - scalar;

  EXPECT_EQ(res._data, expected._data);
}

// Vector-matrix multiplication tests
TYPED_TEST(GTestVectorArithmetic, op_mul_matrix__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto vecResult = Vector<value_type, 2>::FromList({
    3, 4
  });
  ASSERT_TRUE(vecResult.has_value());
  const auto vec = vecResult.value();
  auto matResult = Matrix<value_type, 2, 3, true>::FromList({
    {1, 2, 3},
    {4, 5, 6}
  });
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  auto expectedResult = Vector<value_type, 3>::FromList({
    19, 26, 33  // 3*1 + 4*4, 3*2 + 4*5, 3*3 + 4*6
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT - vector * matrix multiplication
  const auto res = vec.transpose() * mat;

  EXPECT_EQ(res._data, expected._data);
}

// Element-wise operations tests
TYPED_TEST(GTestVectorArithmetic, op_plus_equals__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto aResult = Vector<value_type, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(aResult.has_value());
  auto a = aResult.value();
  auto bResult = Vector<value_type, 3>::FromList({
    4, 5, 6
  });
  ASSERT_TRUE(bResult.has_value());
  const auto b = bResult.value();
  auto expectedResult = Vector<value_type, 3>::FromList({
    5, 7, 9
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT
  a += b;

  EXPECT_EQ(a._data, expected._data);
}

TYPED_TEST(GTestVectorArithmetic, op_minus_equals__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto aResult = Vector<value_type, 3>::FromList({
    5, 7, 9
  });
  ASSERT_TRUE(aResult.has_value());
  auto a = aResult.value();
  auto bResult = Vector<value_type, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(bResult.has_value());
  const auto b = bResult.value();
  auto expectedResult = Vector<value_type, 3>::FromList({
    4, 5, 6
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT
  a -= b;

  EXPECT_EQ(a._data, expected._data);
}

TYPED_TEST(GTestVectorArithmetic, op_mul_equals_scalar__Success) // NOLINT
{
  using value_type = typename TypeParam::type;
  // clang-format off
  auto aResult = Vector<value_type, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(aResult.has_value());
  auto a = aResult.value();
  const value_type scalar = 3;
  auto expectedResult = Vector<value_type, 3>::FromList({
    3, 6, 9
  });
  ASSERT_TRUE(expectedResult.has_value());
  const auto expected = expectedResult.value();
  // clang-format on

  // call UUT
  a *= scalar;

  EXPECT_EQ(a._data, expected._data);
}

// Edge case tests
TEST(Vector, ZeroVector_norm__Success) // NOLINT
{
  // clang-format off
  const auto zero_vec = Vector<float32, 3>::Zeros();
  // clang-format on

  // call UUT
  const auto norm_sq = zero_vec.normSq();
  const auto norm    = zero_vec.norm();

  EXPECT_EQ(norm_sq, 0.0F);
  EXPECT_EQ(norm, 0.0F);
}

TEST(Vector, ZeroVector_dot_product__Success) // NOLINT
{
  // clang-format off
  const auto zero_vec = Vector<float32, 3>::Zeros();
  auto other_vecResult = Vector<float32, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(other_vecResult.has_value());
  const auto other_vec = other_vecResult.value();
  // clang-format on

  // call UUT
  const auto dot_product = zero_vec * other_vec;

  EXPECT_EQ(dot_product, 0.0F);
}

TEST(Vector, UnitVector_normalize__Success) // NOLINT
{
  // clang-format off
  const auto unit_vec = Vector<float32, 3>::UnitVector<1>();
  // clang-format on

  // call UUT
  const auto normalized = unit_vec.normalize();

  // Unit vector should remain unchanged after normalization
  EXPECT_EQ(normalized._data, unit_vec._data);
  EXPECT_FLOAT_EQ(unit_vec.norm(), 1.0F);
}

// Error handling tests
TEST(Vector, op_at__FailBadRowIdx) // NOLINT
{
  using VecType = Vector<sint32, 4>;
  auto aResult  = Vector<sint32, 4>::FromList({1, 2, 3, 4});
  ASSERT_TRUE(aResult.has_value());
  const auto a = aResult.value();

  // call UUT with invalid index
  auto retVal = a[VecType::Rows]; // Out of bounds

  EXPECT_FALSE(retVal.has_value());
  EXPECT_EQ(retVal.error(), Errors::invalid_access_row);
}

TEST(Vector, op_divide_by_zero__Fail) // NOLINT
{
  // clang-format off
  auto vecResult = Vector<float32, 3>::FromList({
    1, 2, 3
  });
  ASSERT_TRUE(vecResult.has_value());
  auto vec = vecResult.value();
  // clang-format on

  // call UUT - division by zero
  auto result = vec / 0.0F;

  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), Errors::divide_by_zero);
}
