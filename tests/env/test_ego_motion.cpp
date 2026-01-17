// Copyright (c) 2026 Kilo Code
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "env/ego_motion.hpp" // IWYU pragma: keep

namespace tracking
{
namespace env
{

// Test fixture for ego motion calculations
class EgoMotionTest: public ::testing::Test
{
protected:
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

  EgoMotion<float64>::InertialMotion motion{};
  EgoMotion<float64>::Geometry       geometry{};
  float64                            dt{};
};

// Test linear motion displacement
TEST_F(EgoMotionTest, LinearMotionDisplacement__Success)
{
  auto egoMotion = EgoMotion<float64>{motion, geometry, dt};

  // Expected displacement values (from derivation)
  const float64 expected_dx   = 1.009983166750833e+00; // T*v + 0.5*T²*(a - v*w)
  const float64 expected_dy   = 5.049957916806945e-03; // 0.5*T²*v*w
  const float64 expected_dpsi = 1.000000000000000e-02; // T*w

  // Expected covariance matrix (from J * Pin * J^T)
  // clang-format off
  const auto expected_cov = math::SquareMatrix<float64, 3, true>::FromList({
  {+2.501062500000000e-03, -6.250000000000321e-08, -2.500000000000001e-05},
  {-6.250000000000321e-08, +6.312500000000004e-06, +1.250000000000001e-05},
  {-2.500000000000001e-05, +1.250000000000001e-05, +2.500000000000001e-05}
  });
  // clang-format on

  // Verify displacement vector
  EXPECT_FLOAT_EQ(egoMotion.getDisplacementCog().vec.at_unsafe(0), expected_dx);
  EXPECT_FLOAT_EQ(egoMotion.getDisplacementCog().vec.at_unsafe(1), expected_dy);
  EXPECT_FLOAT_EQ(egoMotion.getDisplacementCog().vec.at_unsafe(2), expected_dpsi);

  // Verify covariance matrix
  auto actual_cov = egoMotion.getDisplacementCog().cov();
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_FLOAT_EQ(actual_cov.at_unsafe(i, j), expected_cov.at_unsafe(i, j));
    }
  }
}

// Test circular motion displacement
TEST_F(EgoMotionTest, CircularMotionDisplacement__Success)
{
  // Use larger ω for circular motion
  motion.w = 0.5; // 0.5 rad/s

  auto egoMotion = EgoMotion<float64>{motion, geometry, dt};

  // Expected displacement values (from circular motion equations)
  const float64 expected_dx   = 1.009579219267702e+00;
  const float64 expected_dy   = 2.524474002168182e-02;
  const float64 expected_dpsi = 5.000000000000000e-02;

  // Verify displacement vector
  EXPECT_FLOAT_EQ(egoMotion.getDisplacementCog().vec.at_unsafe(0), expected_dx);
  EXPECT_FLOAT_EQ(egoMotion.getDisplacementCog().vec.at_unsafe(1), expected_dy);
  EXPECT_FLOAT_EQ(egoMotion.getDisplacementCog().vec.at_unsafe(2), expected_dpsi);

  // Expected covariance matrix (from J * Pin * J^T)
  // clang-format off
  const auto expected_cov = math::SquareMatrix<float64, 3, true>::FromList({
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
      EXPECT_FLOAT_EQ(actual_cov.at_unsafe(i, j), expected_cov.at_unsafe(i, j));
    }
  }
}

} // namespace env
} // namespace tracking