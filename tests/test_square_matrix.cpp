#include "gtest/gtest.h"

#include "trackingLib/math/linalg/square_matrix.hpp"

TEST(SquareMatrix, decomposeLDLT)
{
  // clang-format off
  tracking::math::SquareMatrix<float32, 6> cov({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  // clang-format on

  tracking::math::TriangularMatrix<float32, 6, true> L{};
  tracking::math::DiagonalMatrix<float32, 6>         D{};

  // call UUT
  cov.decomposeLDLT(L, D);

  tracking::math::SquareMatrix<float32, 6> recomposed = L * D * L.transpose();
  for (auto row = 0; row < 6; row++)
  {
    for (auto col = 0; col < 6; col++)
    {
      EXPECT_FLOAT_EQ(cov(row,col), recomposed(row,col));
    }
  }
}
