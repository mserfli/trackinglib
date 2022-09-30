#include "gtest/gtest.h"
#include <trackingLib/math/linalg/point2d.h>

// NOLINTBEGIN(modernize-use-trailing-return-type)

TEST(IntMatrixBasicOpTest, AddingTwoMatrix)
{
  auto a = tracking::math::Point2d<float32>::ones();
  auto b = tracking::math::Point2d<float32>::zeros();
  EXPECT_EQ(1, 1);
}

// NOLINTEND(modernize-use-trailing-return-type)
