#include "gtest/gtest.h"
#include "trackingLib/math/linalg/point2d.h" // IWYU pragma: keep

TEST(Point2d, ctor_default) // NOLINT
{
  using Point2d = tracking::math::Point2d<float32>;
  const auto p  = Point2d{};
  EXPECT_FLOAT_EQ(p.x(), 0.0F);
  EXPECT_FLOAT_EQ(p.y(), 0.0F);
}

TEST(Point2d, ctor_from_values) // NOLINT
{
  using Point2d = tracking::math::Point2d<float32>;
  const auto p  = Point2d::FromValues(1.0F, 2.0F);
  EXPECT_FLOAT_EQ(p.x(), 1.0F);
  EXPECT_FLOAT_EQ(p.y(), 2.0F);
}

TEST(Point2d, accessors) // NOLINT
{
  using Point2d = tracking::math::Point2d<float32>;
  auto p        = Point2d::FromValues(1.0F, 2.0F);

  p.x() = 3.0F;
  p.y() = 4.0F;

  EXPECT_FLOAT_EQ(p.x(), 3.0F);
  EXPECT_FLOAT_EQ(p.y(), 4.0F);
}

TEST(Point2d, inheritance) // NOLINT
{
  using Point2d = tracking::math::Point2d<float32>;
  const auto p  = Point2d::FromValues(3.0F, 4.0F);

  // Check Vector functionality
  EXPECT_FLOAT_EQ(p.norm(), 5.0F);
}
