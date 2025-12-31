#include "gtest/gtest.h"
#include "trackingLib/math/linalg/point3d.h" // IWYU pragma: keep

TEST(Point3d, ctor_default) // NOLINT
{
  using Point3d = tracking::math::Point3d<float32>;
  const auto p  = Point3d{};
  EXPECT_FLOAT_EQ(p.x(), 0.0F);
  EXPECT_FLOAT_EQ(p.y(), 0.0F);
  EXPECT_FLOAT_EQ(p.z(), 0.0F);
}

TEST(Point3d, ctor_from_values) // NOLINT
{
  using Point3d = tracking::math::Point3d<float32>;
  const auto p  = Point3d::FromValues(1.0F, 2.0F, 3.0F);
  EXPECT_FLOAT_EQ(p.x(), 1.0F);
  EXPECT_FLOAT_EQ(p.y(), 2.0F);
  EXPECT_FLOAT_EQ(p.z(), 3.0F);
}

TEST(Point3d, accessors) // NOLINT
{
  using Point3d = tracking::math::Point3d<float32>;
  auto p        = Point3d::FromValues(1.0F, 2.0F, 3.0F);

  p.x() = 4.0F;
  p.y() = 5.0F;
  p.z() = 6.0F;

  EXPECT_FLOAT_EQ(p.x(), 4.0F);
  EXPECT_FLOAT_EQ(p.y(), 5.0F);
  EXPECT_FLOAT_EQ(p.z(), 6.0F);
}

TEST(Point3d, inheritance) // NOLINT
{
  using Point3d = tracking::math::Point3d<float32>;
  // 3, 4, 0 -> norm 5
  const auto p = Point3d::FromValues(3.0F, 4.0F, 0.0F);

  // Check Vector functionality
  EXPECT_FLOAT_EQ(p.norm(), 5.0F);
}
