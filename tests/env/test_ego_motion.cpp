// Copyright (c) 2026 Kilo Code
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "env/ego_motion.hpp"                         // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include <limits>

namespace tracking
{
namespace env
{

// Typed test fixture for ego motion calculations with different covariance matrix types
template <typename CovarianceMatrixPolicy_>
class GTestEgoMotion: public ::testing::Test
{
protected:
  using value_type    = typename CovarianceMatrixPolicy_::value_type;
  using EgoMotionType = EgoMotion<CovarianceMatrixPolicy_>;

  typename EgoMotionType::InertialMotion motion{};
  typename EgoMotionType::Geometry       geometry{};
  value_type                             dt{};
  const value_type                       epsilon{1.1 * std::numeric_limits<value_type>::epsilon()};

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
    const value_type expected_dx   = 1.009983166750833e+00; // T*v + 0.5*T²*(a - v*w)
    const value_type expected_dy   = 5.049957916806945e-03; // 0.5*T²*v*w
    const value_type expected_dpsi = 1.000000000000000e-02; // T*w

    // Expected covariance matrix (from J * Pin * J^T)
    // clang-format off
    const auto expected_cov = math::SquareMatrix<value_type, 3, true>::FromList({
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

  void test_GetVelocityAt_ZeroYawRate__ReducesToPureTranslation()
  {
    motion.w = 0.0;

    auto egoMotion = EgoMotionType{motion, geometry, dt};

    // compensatePosition maps ego->COG via += distCog2Ego, so the COG is at ego-x = -distCog2Ego.
    const auto velAtCog   = egoMotion.getVelocityAt(-geometry.distCog2Ego, 0.0);
    const auto velAtOther = egoMotion.getVelocityAt(-geometry.distCog2Ego + 3.0, 2.0);

    EXPECT_NEAR(velAtCog.x(), motion.v, epsilon);
    EXPECT_NEAR(velAtCog.y(), 0.0, epsilon);
    EXPECT_NEAR(velAtOther.x(), motion.v, epsilon);
    EXPECT_NEAR(velAtOther.y(), 0.0, epsilon);
  }

  void test_GetVelocityAt_ZeroMountOffset__ReducesToCogVelocity()
  {
    auto egoMotion = EgoMotionType{motion, geometry, dt};

    // compensatePosition maps ego->COG via += distCog2Ego, so the COG is at ego-x = -distCog2Ego.
    const auto velAtCog = egoMotion.getVelocityAt(-geometry.distCog2Ego, 0.0);

    EXPECT_NEAR(velAtCog.x(), motion.v, epsilon);
    EXPECT_NEAR(velAtCog.y(), 0.0, epsilon);
  }

  void test_GetVelocityAt_NonzeroYawRateAndOffset__AppliesLeverArmCrossProduct()
  {
    auto egoMotion = EgoMotionType{motion, geometry, dt};

    // COG-relative offset rx = mountX + distCog2Ego, so pick mountX to land rx = 2.0.
    const value_type mountX = -geometry.distCog2Ego + 2.0; // rx = 2.0
    const value_type mountY = 3.0;                         // ry = 3.0

    const auto vel = egoMotion.getVelocityAt(mountX, mountY);

    // v_point = (v, 0) + w x r = (v - w*ry, w*rx)
    const value_type tol = 10 * epsilon;
    EXPECT_NEAR(vel.x(), motion.v - (motion.w * 3.0), tol);
    EXPECT_NEAR(vel.y(), motion.w * 2.0, tol);
  }

  // The following GetVelocityAt tests derive the COG location independently from compensatePosition (which establishes
  // the COG at ego-x + distCog2Ego), NOT from getVelocityAt's own convention. They assert the
  // mathematically correct behavior, so a genuine sign bug in getVelocityAt makes them fail (RED).
  void test_GetVelocityAt_AtCogFromCompensationConvention__ZeroLeverArmVelocity()
  {
    // motion.w is nonzero (0.1 from SetUp). compensatePosition maps ego->COG via += distCog2Ego,
    // so the COG sits at ego-x = -distCog2Ego. A rigid point at the COG has no yaw-induced lateral
    // velocity, only the forward speed v.
    auto egoMotion = EgoMotionType{motion, geometry, dt};

    const auto velAtCog = egoMotion.getVelocityAt(-geometry.distCog2Ego, 0.0);

    const value_type tol = 10 * epsilon;
    EXPECT_NEAR(velAtCog.x(), motion.v, tol);
    EXPECT_NEAR(velAtCog.y(), 0.0, tol); // current code gives w*(-2*distCog2Ego) != 0
  }

  void test_GetVelocityAt_LeverArmSignConsistentWithCompensatePosition__Success()
  {
    auto egoMotion = EgoMotionType{motion, geometry, dt};

    const value_type tol = 10 * epsilon;

    // compensatePosition maps ego->COG via += distCog2Ego, so the COG-relative x of an ego-frame
    // point px is (px + distCog2Ego): v_point = (v - w*ry, w*(px + distCog2Ego)).
    // Point A: ego (px, py) = (2.0, 3.0)
    {
      const value_type px  = 2.0;
      const value_type py  = 3.0;
      const auto       vel = egoMotion.getVelocityAt(px, py);
      EXPECT_NEAR(vel.x(), motion.v - (motion.w * py), tol);
      EXPECT_NEAR(vel.y(), motion.w * (px + geometry.distCog2Ego), tol); // current code yields w*(px - distCog2Ego)
    }
    // Point B: ego (px, py) = (-4.0, 1.5)
    {
      const value_type px  = -4.0;
      const value_type py  = 1.5;
      const auto       vel = egoMotion.getVelocityAt(px, py);
      EXPECT_NEAR(vel.x(), motion.v - (motion.w * py), tol);
      EXPECT_NEAR(vel.y(), motion.w * (px + geometry.distCog2Ego), tol);
    }
  }

  void test_GetVelocityAt_CogIsFixedPointOfPureRotation__ConsistentWithVelocity()
  {
    // Pure rotation (v=0, a=0, w!=0): calcDisplacementVector yields zero translation, so
    // compensatePosition is a pure rotation about the COG. The rotation's fixed point (the ego
    // point that maps to itself) is therefore the COG; getVelocityAt at that same point must
    // return zero. This ties the two functions together with no hand-chosen sign.
    motion.v = 0.0;
    motion.a = 0.0;
    motion.w = 0.1;

    auto egoMotion = EgoMotionType{motion, geometry, dt};

    // The fixed point of the pure rotation, resolved from compensatePosition alone.
    const value_type cogEgoX = -geometry.distCog2Ego;
    const value_type cogEgoY = 0.0;

    value_type outX{};
    value_type outY{};
    egoMotion.compensatePosition(outX, outY, cogEgoX, cogEgoY);

    const value_type tol = 10 * epsilon;
    EXPECT_NEAR(outX, cogEgoX, tol); // confirm it is the rotation fixed point
    EXPECT_NEAR(outY, cogEgoY, tol);

    // The COG (= rotation fixed point) must have zero velocity per getVelocityAt.
    const auto velAtCog = egoMotion.getVelocityAt(cogEgoX, cogEgoY);
    EXPECT_NEAR(velAtCog.x(), 0.0, tol); // v = 0
    EXPECT_NEAR(velAtCog.y(), 0.0, tol); // no lever arm at the COG
  }

  void test_CircularMotionDisplacement__Success()
  {
    // Use larger ω for circular motion
    motion.w = 0.5; // 0.5 rad/s

    auto egoMotion = EgoMotionType{motion, geometry, dt};

    // Expected displacement values (from circular motion equations)
    const value_type expected_dx   = 1.009579219267702e+00;
    const value_type expected_dy   = 2.524474002168182e-02;
    const value_type expected_dpsi = 5.000000000000000e-02;

    // Verify displacement vector
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(0), expected_dx, epsilon);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(1), expected_dy, epsilon);
    EXPECT_NEAR(egoMotion.getDisplacementCog().vec.at_unsafe(2), expected_dpsi, epsilon);

    // Expected covariance matrix (from J * Pin * J^T)
    // clang-format off
    const auto expected_cov = math::SquareMatrix<value_type, 3, true>::FromList({
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
using CovarianceMatrixTypes = ::testing::Types<math::FullCovarianceMatrixPolicy<float32>,
                                               math::FullCovarianceMatrixPolicy<float64>,
                                               math::FactoredCovarianceMatrixPolicy<float32>,
                                               math::FactoredCovarianceMatrixPolicy<float64>>;

TYPED_TEST_SUITE(GTestEgoMotion, CovarianceMatrixTypes);

TYPED_TEST(GTestEgoMotion, LinearMotionDisplacement__Success)
{
  GTestEgoMotion<TypeParam>::test_LinearMotionDisplacement__Success();
}

TYPED_TEST(GTestEgoMotion, CircularMotionDisplacement__Success)
{
  GTestEgoMotion<TypeParam>::test_CircularMotionDisplacement__Success();
}

TYPED_TEST(GTestEgoMotion, GetVelocityAt_ZeroYawRate__ReducesToPureTranslation)
{
  GTestEgoMotion<TypeParam>::test_GetVelocityAt_ZeroYawRate__ReducesToPureTranslation();
}

TYPED_TEST(GTestEgoMotion, GetVelocityAt_ZeroMountOffset__ReducesToCogVelocity)
{
  GTestEgoMotion<TypeParam>::test_GetVelocityAt_ZeroMountOffset__ReducesToCogVelocity();
}

TYPED_TEST(GTestEgoMotion, GetVelocityAt_NonzeroYawRateAndOffset__AppliesLeverArmCrossProduct)
{
  GTestEgoMotion<TypeParam>::test_GetVelocityAt_NonzeroYawRateAndOffset__AppliesLeverArmCrossProduct();
}

TYPED_TEST(GTestEgoMotion, GetVelocityAt_AtCogFromCompensationConvention__ZeroLeverArmVelocity)
{
  GTestEgoMotion<TypeParam>::test_GetVelocityAt_AtCogFromCompensationConvention__ZeroLeverArmVelocity();
}

TYPED_TEST(GTestEgoMotion, GetVelocityAt_LeverArmSignConsistentWithCompensatePosition__Success)
{
  GTestEgoMotion<TypeParam>::test_GetVelocityAt_LeverArmSignConsistentWithCompensatePosition__Success();
}

TYPED_TEST(GTestEgoMotion, GetVelocityAt_CogIsFixedPointOfPureRotation__ConsistentWithVelocity)
{
  GTestEgoMotion<TypeParam>::test_GetVelocityAt_CogIsFixedPointOfPureRotation__ConsistentWithVelocity();
}

} // namespace env
} // namespace tracking