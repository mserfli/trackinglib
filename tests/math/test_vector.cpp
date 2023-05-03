#include "gtest/gtest.h"
#include "trackingLib/math/linalg/vector.hpp"
#include <type_traits>

TEST(Vector, ctor_initializerList__Success) // NOLINT
{
  using VecType = tracking::math::Vector<sint32, 4>;
  const auto a = VecType::Ones();
  
  // call UUT
  const auto b = VecType::FromList({1,1,1,1});
  
  EXPECT_EQ(a, b);
}

TEST(Vector, UnitVector__Success) // NOLINT
{
  using VecType = tracking::math::Vector<sint32, 4>;
  const auto a = VecType::FromList({0,0,1,0});

  // call UUT
  const auto res = VecType::UnitVector<2>();

  EXPECT_EQ(a, res);
}

TEST(Vector, op_dot__Success) // NOLINT
{
  using VecType = tracking::math::Vector<sint32, 4>;
  const auto a = VecType::Ones();
  const auto b = VecType::FromList({1,2,3,4});
  
  // call UUT
  const auto res = a * b;

  EXPECT_EQ(res, 10);
}

TEST(Vector, op_normSq__Success) // NOLINT
{
  using VecType = tracking::math::Vector<sint32, 4>;
  const auto a = VecType::FromList({1,2,3,4});
  
  // call UUT
  const auto res = a.normSq();

  EXPECT_EQ(res, 30);
}

TEST(Vector, op_norm__Success) // NOLINT
{
  using VecType = tracking::math::Vector<float32, 2>;
  const auto a = VecType::FromList({3,4});
  
  // call UUT
  const auto res = a.norm();

  EXPECT_FLOAT_EQ(res, 5.0F);
}

TEST(Vector, op_normalize__Success) // NOLINT
{
  using VecType = tracking::math::Vector<float32, 2>;
  auto a = VecType::FromList({3,4});
  const auto b = VecType::FromList({3/5.0,4/5.0});
  
  // call UUT
  a.normalize();

  EXPECT_EQ(a, b);
}
