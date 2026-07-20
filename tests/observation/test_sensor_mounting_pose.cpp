#include <gtest/gtest.h>

#include "trackingLib/math/linalg/matrix.h"
#include "trackingLib/observation/sensor_mounting_pose.h"
#include <cmath>

using Testvalue_type = float64;
using Pose           = tracking::observation::SensorMountingPose<Testvalue_type>;

namespace
{
const Testvalue_type kPi  = std::acos(static_cast<Testvalue_type>(-1.0));
const Testvalue_type kTol = 1e-12;
} // namespace

TEST(SensorMountingPose, defaultCtor__IsIdentity) // NOLINT
{
  const Pose pose{};

  EXPECT_NEAR(pose.tx(), 0.0, kTol);
  EXPECT_NEAR(pose.ty(), 0.0, kTol);
  EXPECT_NEAR(pose.cosYaw(), 1.0, kTol);
  EXPECT_NEAR(pose.sinYaw(), 0.0, kTol);
}

TEST(SensorMountingPose, positionToSensorFrame__IdentityIsNoOp) // NOLINT
{
  const Pose pose{};

  const auto ps = pose.positionToSensorFrame(3.0, -2.0);

  EXPECT_NEAR(ps.x(), 3.0, kTol);
  EXPECT_NEAR(ps.y(), -2.0, kTol);
}

TEST(SensorMountingPose, directionToSensorFrame__IdentityIsNoOp) // NOLINT
{
  const Pose pose{};

  const auto vs = pose.directionToSensorFrame(1.5, 4.0);

  EXPECT_NEAR(vs.x(), 1.5, kTol);
  EXPECT_NEAR(vs.y(), 4.0, kTol);
}

TEST(SensorMountingPose, positionToSensorFrame__PureTranslation) // NOLINT
{
  const auto pose = Pose::FromValues(1.0, 2.0, 0.0);

  const auto ps = pose.positionToSensorFrame(4.0, 6.0);

  EXPECT_NEAR(ps.x(), 3.0, kTol);
  EXPECT_NEAR(ps.y(), 4.0, kTol);
}

TEST(SensorMountingPose, directionToSensorFrame__PureTranslationIsNoOp) // NOLINT
{
  // static mount: translation has no lever-arm effect on directions
  const auto pose = Pose::FromValues(1.0, 2.0, 0.0);

  const auto vs = pose.directionToSensorFrame(4.0, 6.0);

  EXPECT_NEAR(vs.x(), 4.0, kTol);
  EXPECT_NEAR(vs.y(), 6.0, kTol);
}

TEST(SensorMountingPose, positionToSensorFrame__PureYawRotation) // NOLINT
{
  // sensor mounted with +90deg yaw relative to tracking frame
  const auto pose = Pose::FromValues(0.0, 0.0, kPi / 2.0);

  const auto ps = pose.positionToSensorFrame(1.0, 0.0);

  EXPECT_NEAR(ps.x(), 0.0, 1e-9);
  EXPECT_NEAR(ps.y(), -1.0, 1e-9);
}

TEST(SensorMountingPose, directionToSensorFrame__PureYawRotation) // NOLINT
{
  const auto pose = Pose::FromValues(0.0, 0.0, kPi / 2.0);

  const auto vs = pose.directionToSensorFrame(0.0, 1.0);

  EXPECT_NEAR(vs.x(), 1.0, 1e-9);
  EXPECT_NEAR(vs.y(), 0.0, 1e-9);
}

TEST(SensorMountingPose, positionToSensorFrame__CombinedTranslationAndYaw) // NOLINT
{
  const auto pose = Pose::FromValues(1.0, 0.0, kPi / 2.0);

  const auto ps = pose.positionToSensorFrame(1.0, 1.0);

  // translate: (0, 1), then rotate by -90deg: (1, 0)
  EXPECT_NEAR(ps.x(), 1.0, 1e-9);
  EXPECT_NEAR(ps.y(), 0.0, 1e-9);
}

TEST(SensorMountingPose, rotateJacobianColumns__IdentityIsNoOp) // NOLINT
{
  const Pose pose{};

  tracking::math::Matrix<Testvalue_type, 2, 4> jacobian{};
  jacobian.at_unsafe(0, 0) = 1.0;
  jacobian.at_unsafe(0, 2) = 2.0;
  jacobian.at_unsafe(1, 0) = 3.0;
  jacobian.at_unsafe(1, 2) = 4.0;

  pose.rotateJacobianColumns(jacobian, 0, 2);

  EXPECT_NEAR(jacobian.at_unsafe(0, 0), 1.0, kTol);
  EXPECT_NEAR(jacobian.at_unsafe(0, 2), 2.0, kTol);
  EXPECT_NEAR(jacobian.at_unsafe(1, 0), 3.0, kTol);
  EXPECT_NEAR(jacobian.at_unsafe(1, 2), 4.0, kTol);
}

TEST(SensorMountingPose, rotateJacobianColumns__PostMultipliesLocalPartialsByRotationTranspose) // NOLINT
{
  // rotateJacobianColumns(H, colX, colY) computes the row-vector chain rule [Lx,Ly] * R(theta)^T:
  //   H(r,colX) = Lx*cos(theta) - Ly*sin(theta), H(r,colY) = Lx*sin(theta) + Ly*cos(theta)
  const auto pose = Pose::FromValues(0.0, 0.0, kPi / 3.0);

  tracking::math::Matrix<Testvalue_type, 1, 2> jacobian{};
  const Testvalue_type                         lx = 1.0;
  const Testvalue_type                         ly = 0.0;
  jacobian.at_unsafe(0, 0)                        = lx;
  jacobian.at_unsafe(0, 1)                        = ly;

  pose.rotateJacobianColumns(jacobian, 0, 1);

  const Testvalue_type cosYaw = pose.cosYaw();
  const Testvalue_type sinYaw = pose.sinYaw();
  EXPECT_NEAR(jacobian.at_unsafe(0, 0), (lx * cosYaw) - (ly * sinYaw), 1e-9);
  EXPECT_NEAR(jacobian.at_unsafe(0, 1), (lx * sinYaw) + (ly * cosYaw), 1e-9);
}

TEST(SensorMountingPose, positionToSensorFrame__RoundTripsWithInverse) // NOLINT
{
  const auto pose = Pose::FromValues(2.0, -3.0, 0.7);

  const auto ps = pose.positionToSensorFrame(5.0, 1.0);

  // invert: rotate ps back by +theta, then add back the translation
  const auto cosYaw = pose.cosYaw();
  const auto sinYaw = pose.sinYaw();
  const auto px     = (cosYaw * ps.x()) - (sinYaw * ps.y()) + pose.tx();
  const auto py     = (sinYaw * ps.x()) + (cosYaw * ps.y()) + pose.ty();

  EXPECT_NEAR(px, 5.0, 1e-9);
  EXPECT_NEAR(py, 1.0, 1e-9);
}
