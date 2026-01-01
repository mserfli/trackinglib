#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_column_view.hpp"             // IWYU pragma: keep
#include "trackingLib/math/linalg/vector.hpp"                         // IWYU pragma: keep

using namespace tracking::math;

TEST(Vector, UnitVector__Success) // NOLINT
{
  using VecType = Vector<sint32, 4>;
  const auto a  = conversions::VectorFromList<sint32, 4>({0, 0, 1, 0});

  // call UUT
  const auto res = VecType::UnitVector<2>();

  EXPECT_EQ(a, res);
}

TEST(Vector, test_op_at_Success) // NOLINT
{
  using VecType = Vector<sint32, 4>;
  auto a        = conversions::VectorFromList<sint32, 4>({1, 2, 3, 4});

  // call UUT
  auto retVal = a[VecType::Rows - 1];

  EXPECT_TRUE(retVal.has_value());
  EXPECT_EQ(retVal, 4);
}

TEST(Vector, test_op_at_const_Success) // NOLINT
{
  using VecType = Vector<sint32, 4>;
  const auto a  = conversions::VectorFromList<sint32, 4>({1, 2, 3, 4});

  // call UUT
  auto retVal = a[VecType::Rows - 1];

  EXPECT_TRUE(retVal.has_value());
  EXPECT_EQ(retVal, 4);
}

TEST(Vector, op_dot__Success) // NOLINT
{
  using VecType = Vector<sint32, 4>;
  const auto a  = VecType::Ones();
  const auto b  = conversions::VectorFromList<sint32, 4>({1, 2, 3, 4});

  // call UUT
  const auto res = a * b;

  EXPECT_EQ(res, 10);
}

TEST(Vector, op_normSq__Success) // NOLINT
{
  const auto a = conversions::VectorFromList<sint32, 4>({1, 2, 3, 4});

  // call UUT
  const auto res = a.normSq();

  EXPECT_EQ(res, 30);
}

TEST(Vector, op_norm__Success) // NOLINT
{
  const auto a = conversions::VectorFromList<float32, 2>({3, 4});

  // call UUT
  const auto res = a.norm();

  EXPECT_FLOAT_EQ(res, 5.0F);
}

TEST(Vector, op_normalize__Success) // NOLINT
{
  const auto a = conversions::VectorFromList<float32, 2>({3, 4});
  const auto b = conversions::VectorFromList<float32, 2>({3 / 5.0, 4 / 5.0});

  // call UUT
  const auto res = a.normalize();

  EXPECT_EQ(res, b);
}
