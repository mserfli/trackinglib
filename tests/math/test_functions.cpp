#include "gtest/gtest.h"
#include "trackingLib/math/analysis/functions.h"

/// \file test_functions.cpp
/// \brief Unit tests for compile-time mathematical functions
///
/// This file contains comprehensive tests for the compile-time power function
/// and other mathematical functions in the analysis module.

namespace tracking
{
namespace math
{
namespace
{

// Test fixture for functions tests
class GTestFunctions: public ::testing::Test
{
protected:
  // Common test values
  static constexpr double kEpsilon    = 1e-10;
  static constexpr int    kIntBase    = 2;
  static constexpr double kDoubleBase = 2.5;
  static constexpr float  kFloatBase  = 1.5f;
};

// Test compile-time power function with integer types
TEST_F(GTestFunctions, pow_IntBase_Exponent0__Success)
{
  constexpr int result = pow<0>(kIntBase);
  EXPECT_EQ(result, 1); // x^0 = 1 for any x
}

TEST_F(GTestFunctions, pow_IntBase_Exponent1__Success)
{
  constexpr int result = pow<1>(kIntBase);
  EXPECT_EQ(result, kIntBase); // x^1 = x
}

TEST_F(GTestFunctions, pow_IntBase_Exponent2__Success)
{
  constexpr int result = pow<2>(kIntBase);
  EXPECT_EQ(result, 4); // 2^2 = 4
}

TEST_F(GTestFunctions, pow_IntBase_Exponent3__Success)
{
  constexpr int result = pow<3>(kIntBase);
  EXPECT_EQ(result, 8); // 2^3 = 8
}

TEST_F(GTestFunctions, pow_IntBase_Exponent5__Success)
{
  constexpr int result = pow<5>(kIntBase);
  EXPECT_EQ(result, 32); // 2^5 = 32
}

// Test compile-time power function with double types
TEST_F(GTestFunctions, pow_DoubleBase_Exponent0__Success)
{
  constexpr double result = pow<0>(kDoubleBase);
  EXPECT_NEAR(result, 1.0, kEpsilon); // x^0 = 1 for any x
}

TEST_F(GTestFunctions, pow_DoubleBase_Exponent1__Success)
{
  constexpr double result = pow<1>(kDoubleBase);
  EXPECT_NEAR(result, kDoubleBase, kEpsilon); // x^1 = x
}

TEST_F(GTestFunctions, pow_DoubleBase_Exponent2__Success)
{
  constexpr double result = pow<2>(kDoubleBase);
  EXPECT_NEAR(result, 6.25, kEpsilon); // 2.5^2 = 6.25
}

TEST_F(GTestFunctions, pow_DoubleBase_Exponent3__Success)
{
  constexpr double result = pow<3>(kDoubleBase);
  EXPECT_NEAR(result, 15.625, kEpsilon); // 2.5^3 = 15.625
}

TEST_F(GTestFunctions, pow_DoubleBase_Exponent4__Success)
{
  constexpr double result = pow<4>(kDoubleBase);
  EXPECT_NEAR(result, 39.0625, kEpsilon); // 2.5^4 = 39.0625
}

// Test compile-time power function with float types
TEST_F(GTestFunctions, pow_FloatBase_Exponent0__Success)
{
  constexpr float result = pow<0>(kFloatBase);
  EXPECT_NEAR(result, 1.0f, kEpsilon); // x^0 = 1 for any x
}

TEST_F(GTestFunctions, pow_FloatBase_Exponent1__Success)
{
  constexpr float result = pow<1>(kFloatBase);
  EXPECT_NEAR(result, kFloatBase, kEpsilon); // x^1 = x
}

TEST_F(GTestFunctions, pow_FloatBase_Exponent2__Success)
{
  constexpr float result = pow<2>(kFloatBase);
  EXPECT_NEAR(result, 2.25f, kEpsilon); // 1.5^2 = 2.25
}

TEST_F(GTestFunctions, pow_FloatBase_Exponent3__Success)
{
  constexpr float result = pow<3>(kFloatBase);
  EXPECT_NEAR(result, 3.375f, kEpsilon); // 1.5^3 = 3.375
}

TEST_F(GTestFunctions, pow_FloatBase_Exponent4__Success)
{
  constexpr float result = pow<4>(kFloatBase);
  EXPECT_NEAR(result, 5.0625f, kEpsilon); // 1.5^4 = 5.0625
}

// Test constexpr evaluation in template parameters
TEST_F(GTestFunctions, pow_TemplateParameter__Success)
{
  // This test verifies that the pow function can be used in template parameters
  // which requires constexpr evaluation
  constexpr int exponent = 3;
  constexpr int base     = 2;
  constexpr int result   = pow<exponent>(base);

  // Use the result in a template parameter to ensure it's truly constexpr
  std::array<int, result> test_array;
  EXPECT_EQ(test_array.size(), 8); // 2^3 = 8
}

// Test edge cases
TEST_F(GTestFunctions, pow_NegativeBase_PositiveExponent__Success)
{
  constexpr double result = pow<3>(-2.0);
  EXPECT_NEAR(result, -8.0, kEpsilon); // (-2)^3 = -8
}

TEST_F(GTestFunctions, pow_NegativeBase_EvenExponent__Success)
{
  constexpr double result = pow<4>(-2.0);
  EXPECT_NEAR(result, 16.0, kEpsilon); // (-2)^4 = 16
}

TEST_F(GTestFunctions, pow_ZeroBase_PositiveExponent__Success)
{
  constexpr double result = pow<5>(0.0);
  EXPECT_NEAR(result, 0.0, kEpsilon); // 0^5 = 0
}

TEST_F(GTestFunctions, pow_ZeroBase_ZeroExponent__Success)
{
  // Note: 0^0 is mathematically undefined, but conventionally treated as 1
  // in many programming contexts
  constexpr double result = pow<0>(0.0);
  EXPECT_NEAR(result, 1.0, kEpsilon);
}

TEST_F(GTestFunctions, pow_OneBase_VariousExponents__Success)
{
  constexpr double result1 = pow<0>(1.0);
  constexpr double result2 = pow<1>(1.0);
  constexpr double result3 = pow<5>(1.0);

  EXPECT_NEAR(result1, 1.0, kEpsilon); // 1^0 = 1
  EXPECT_NEAR(result2, 1.0, kEpsilon); // 1^1 = 1
  EXPECT_NEAR(result3, 1.0, kEpsilon); // 1^5 = 1
}

// Test larger exponents to verify the iterative approach works correctly
TEST_F(GTestFunctions, pow_LargeExponent__Success)
{
  constexpr double result = pow<10>(2.0);
  EXPECT_NEAR(result, 1024.0, kEpsilon); // 2^10 = 1024
}

} // namespace
} // namespace math
} // namespace tracking