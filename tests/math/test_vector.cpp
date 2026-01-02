#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"   // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_column_view.hpp"                 // IWYU pragma: keep
#include "trackingLib/math/linalg/vector.hpp"                             // IWYU pragma: keep

using namespace tracking::math;

TEST(Vector, UnitVector__Success) // NOLINT
{
  // clang-format off
  const auto a  = conversions::VectorFromList<sint32, 4>({
    0, 0, 1, 0
  });
  // clang-format on

  // call UUT
  const auto res = Vector<sint32, 4>::UnitVector<2>();

  EXPECT_EQ(a._data, res._data);
}

TEST(Vector, test_op_at_Success) // NOLINT
{
  // clang-format off
  using VecType = Vector<sint32, 4>;
  auto a        = conversions::VectorFromList<sint32, 4>({
    1, 2, 3, 4
  });
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
  const auto a  = conversions::VectorFromList<sint32, 4>({
    1, 2, 3, 4
  });
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
  const auto b  = conversions::VectorFromList<sint32, 4>({
    1, 2, 3, 4
  });
  // clang-format on

  // call UUT
  const auto res = a * b;

  EXPECT_EQ(res, 10);
}

TEST(Vector, op_normSq__Success) // NOLINT
{
  // clang-format off
  const auto a = conversions::VectorFromList<sint32, 4>({
    1, 2, 3, 4
  });
  // clang-format on

  // call UUT
  const auto res = a.normSq();

  EXPECT_EQ(res, 30);
}

TEST(Vector, op_norm__Success) // NOLINT
{
  // clang-format off
  const auto a = conversions::VectorFromList<float32, 2>({
    3, 4
  });
  // clang-format on

  // call UUT
  const auto res = a.norm();

  EXPECT_FLOAT_EQ(res, 5.0F);
}

TEST(Vector, op_normalize__Success) // NOLINT
{
  // clang-format off
  const auto a = conversions::VectorFromList<float32, 2>({
    3, 4
  });
  const auto b = conversions::VectorFromList<float32, 2>({
    3 / 5.0, 4 / 5.0
  });
  // clang-format on

  // call UUT
  const auto res = a.normalize();

  EXPECT_EQ(res._data, b._data);
}
