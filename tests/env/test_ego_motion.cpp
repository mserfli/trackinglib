// Copyright (c) 2026 Kilo Code
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "env/ego_motion.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.h"
#include "math/linalg/covariance_matrix_full.h"
#include <limits>

namespace tracking
{
namespace env
{

// Matrix-Wrapper
struct WrappedCovarianceMatrixFull
{
  template <typename T, sint32 S>
  using Type = math::CovarianceMatrixFull<T, S>;
};

struct WrappedCovarianceMatrixFactored
{
  template <typename T, sint32 S>
  using Type = math::CovarianceMatrixFactored<T, S>;
};

// carrier to bundle MatrixType and FloatType
template <typename MatrixWrapper_, typename FloatType_>
struct TestDefinition
{
  using Wrapper   = MatrixWrapper_;
  using FloatType = FloatType_;
};

// Typed test fixture for ego motion calculations with different covariance matrix types
template <typename T>
class GTestEgoMotion: public ::testing::Test
{
protected:
  using FloatType     = typename T::FloatType;
  using MatrixWrapper = typename T::Wrapper;
  using EgoMotionType = EgoMotion<MatrixWrapper::template Type, FloatType>;

  typename EgoMotionType::InertialMotion motion{};
  typename EgoMotionType::Geometry       geometry{};
  FloatType                              dt{};
  const FloatType                        epsilon{1.1 * std::numeric_limits<FloatType>::epsilon()};

  void SetUp() override
  {
    // Common test setup
    motion.v = 10.0; // m/s
    motion.a = 2.0;  // m/s²
    motion.w = 0.1;  // rad/s (small for linear case)

    motion.sv = 0.50; // m/s std dev
    motion.sa = 0.20; // m/s² std dev
    motion.sw = 0.05; // rad/s std dev

    geometry.distCog2Ego = 1.5; // 1.5m from COG to ego point

    dt = 0.1; // 100ms time step
  }

  void test_LinearMotionDisplacement__Success()
  {
    auto egoMotion = EgoMotionType{motion, geometry, dt};

    // Expected displacement values (from derivation)
    const FloatType expected_dx   = 1.009983166750833e+00; // T*v + 0.5*T²*(a - v*w)
    const FloatType expected_dy   = 5.049957916806945e-03; // 0.5*T²*v*w
    const FloatType expected_dpsi = 1.000000000000000e-02; // T*w

    // Expected covariance matrix (from J * Pin * J^T)
    // clang-format off
    const auto expected_cov = math::SquareMatrix<FloatType, 3, true>::FromList({
      {2.501000000000001e-03,                     0,                     0},
      {                    0, 6.375625000000005e-06, 1.262500000000001e-05},
      {                    0, 1.262500000000001e-05, 2.500000000000001e-05}
    });
    // clang-format on

    // Verify displacement vector
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(0), expected_dx, epsilon);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(1), expected_dy, epsilon);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(2), expected_dpsi, epsilon);

    // Verify covariance matrix
    auto actual_cov = egoMotion.getDisplacementCog().cov();
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        EXPECT_NEAR(actual_cov.at_unsafe(i, j), expected_cov.at_unsafe(i, j), epsilon);
      }
    }
  }

  void test_CircularMotionDisplacement__Success()
  {
    // Use larger ω for circular motion
    motion.w = 0.5; // 0.5 rad/s

    auto egoMotion = EgoMotionType{motion, geometry, dt};

    // Expected displacement values (from circular motion equations)
    const FloatType expected_dx   = 1.009579219267702e+00;
    const FloatType expected_dy   = 2.524474002168182e-02;
    const FloatType expected_dpsi = 5.000000000000000e-02;

    // Verify displacement vector
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(0), expected_dx, epsilon);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(1), expected_dy, epsilon);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(2), expected_dpsi, epsilon);

    // Expected covariance matrix (from J * Pin * J^T)
    // clang-format off
    const auto expected_cov = math::SquareMatrix<FloatType, 3, true>::FromList({
      {+2.498923608418034e-03, +6.227359670916502e-05, -4.207281343930892e-07},
      {+6.227359670916502e-05, +7.930132885369052e-06, +1.261711047085165e-05},
      {-4.207281343930892e-07, +1.261711047085165e-05, +2.500000000000001e-05}
    });
    // clang-format on

    // Verify covariance matrix
    auto actual_cov = egoMotion.getDisplacementCog().cov();
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        EXPECT_NEAR(actual_cov.at_unsafe(i, j), expected_cov.at_unsafe(i, j), epsilon);
      }
    }
  }
};

// Define the types for typed tests
using CovarianceMatrixTypes = ::testing::Types<TestDefinition<WrappedCovarianceMatrixFull, float32>,
                                               TestDefinition<WrappedCovarianceMatrixFull, float64>,
                                               TestDefinition<WrappedCovarianceMatrixFactored, float32>,
                                               TestDefinition<WrappedCovarianceMatrixFactored, float64>>;

TYPED_TEST_SUITE(GTestEgoMotion, CovarianceMatrixTypes);

TYPED_TEST(GTestEgoMotion, LinearMotionDisplacement__Success)
{
  GTestEgoMotion<TypeParam>::test_LinearMotionDisplacement__Success();
}

TYPED_TEST(GTestEgoMotion, CircularMotionDisplacement__Success)
{
  GTestEgoMotion<TypeParam>::test_CircularMotionDisplacement__Success();
}

} // namespace env
} // namespace tracking