#include <gtest/gtest.h>

#include "trackingLib/math/linalg/rotation2d.h"
#include <cmath>

using Testvalue_type = float64;
using Rotation2D     = tracking::math::Rotation2D<Testvalue_type>;

namespace
{
const Testvalue_type kPi  = std::acos(static_cast<Testvalue_type>(-1.0));
const Testvalue_type kTol = 1e-12;
} // namespace

TEST(Rotation2D, defaultCtor__IsIdentity) // NOLINT
{
  const Rotation2D rotation{};

  EXPECT_NEAR(rotation.cos(), 1.0, kTol);
  EXPECT_NEAR(rotation.sin(), 0.0, kTol);
}

TEST(Rotation2D, FromAngle__ComputesCosSin) // NOLINT
{
  const auto rotation = Rotation2D::FromAngle(kPi / 2.0);

  EXPECT_NEAR(rotation.cos(), 0.0, kTol);
  EXPECT_NEAR(rotation.sin(), 1.0, kTol);
}

TEST(Rotation2D, FromCosSin__ReusesGivenValues) // NOLINT
{
  const auto rotation = Rotation2D::FromCosSin(0.6, 0.8);

  EXPECT_NEAR(rotation.cos(), 0.6, kTol);
  EXPECT_NEAR(rotation.sin(), 0.8, kTol);
}

TEST(Rotation2D, apply__IdentityIsNoOp) // NOLINT
{
  const Rotation2D rotation{};

  const auto rotated = rotation.apply(3.0, -2.0);

  EXPECT_NEAR(rotated.x(), 3.0, kTol);
  EXPECT_NEAR(rotated.y(), -2.0, kTol);
}

TEST(Rotation2D, applyTranspose__IdentityIsNoOp) // NOLINT
{
  const Rotation2D rotation{};

  const auto rotated = rotation.applyTranspose(3.0, -2.0);

  EXPECT_NEAR(rotated.x(), 3.0, kTol);
  EXPECT_NEAR(rotated.y(), -2.0, kTol);
}

TEST(Rotation2D, apply__QuarterTurn) // NOLINT
{
  const auto rotation = Rotation2D::FromAngle(kPi / 2.0);

  const auto rotated = rotation.apply(1.0, 0.0);

  EXPECT_NEAR(rotated.x(), 0.0, kTol);
  EXPECT_NEAR(rotated.y(), 1.0, kTol);
}

TEST(Rotation2D, applyTranspose__QuarterTurn) // NOLINT
{
  const auto rotation = Rotation2D::FromAngle(kPi / 2.0);

  const auto rotated = rotation.applyTranspose(1.0, 0.0);

  EXPECT_NEAR(rotated.x(), 0.0, kTol);
  EXPECT_NEAR(rotated.y(), -1.0, kTol);
}

TEST(Rotation2D, applyThenApplyTranspose__RoundTrips) // NOLINT
{
  const auto rotation = Rotation2D::FromAngle(kPi / 5.0);

  const auto forward = rotation.apply(2.0, -3.5);
  const auto back    = rotation.applyTranspose(forward.x(), forward.y());

  EXPECT_NEAR(back.x(), 2.0, kTol);
  EXPECT_NEAR(back.y(), -3.5, kTol);
}
