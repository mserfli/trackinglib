#include "gtest/gtest.h"
#include "trackingLib/math/linalg/square_matrix.hpp"
#include "trackingLib/math/linalg/triangular_matrix.hpp"

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::math::TriangularMatrix<float32, 3, true>;
template class tracking::math::TriangularMatrix<float32, 3, false>;
template auto tracking::math::TriangularMatrix<float32, 3, false>::solve<1>(const tracking::math::Matrix<float32, 3, 1>&) const
    -> Matrix<float32, 3, 1>;
template void tracking::math::TriangularMatrix<float32, 3, false>::setBlock<2, 2, 0, 0, 0, 0>(
    const tracking::math::TriangularMatrix<float32, 2, false>&);


TEST(TriangularMatrix, initFromSquareMatrix_lower)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{1, 0, 0}, 
     {1, 1, 0}, 
     {1, 1, 1}});
  // clang-format on

  auto ones = tracking::math::SquareMatrix<float32, 3>::Ones();

  // call UUT
  tracking::math::TriangularMatrix<float32, 3, true> lowerTriaMat(ones);

  EXPECT_EQ(expMat._data, lowerTriaMat._data);
}

TEST(TriangularMatrix, initFromSquareMatrix_upper)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 1, 1}, 
     {0, 1, 1}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  auto lowerTriaMat = tracking::math::TriangularMatrix<float32, 3, false>(tracking::math::SquareMatrix<float32, 3>::Ones());

  EXPECT_EQ(expMat._data, lowerTriaMat._data);
}

TEST(TriangularMatrix, setBlock_lowerTopLeft)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> lowerTriaMat(
    {{1, 0, 0}, 
     {1, 1, 0}, 
     {1, 1, 1}});
  const tracking::math::TriangularMatrix<float32, 2, true> lowerTriaBlockMat(
    {{2, 0},  
     {4, 3}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{2, 0, 0}, 
     {4, 3, 0}, 
     {1, 1, 1}});
  // clang-format on

  // call UUT
  lowerTriaMat.setBlock<2, 2, 0, 0, 0, 0>(lowerTriaBlockMat);

  EXPECT_EQ(expMat._data, lowerTriaMat._data);
}

TEST(TriangularMatrix, setBlock_lowerBottomLeft)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> lowerTriaMat(
    {{1, 0, 0}, 
     {1, 1, 0}, 
     {1, 1, 1}});
  const tracking::math::TriangularMatrix<float32, 2, true> lowerTriaBlockMat(
    {{2, 0},  
     {4, 3}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{1, 0, 0}, 
     {2, 1, 0}, 
     {4, 3, 1}});
  // clang-format on

  // call UUT
  lowerTriaMat.setBlock<2, 2, 0, 0, 1, 0>(lowerTriaBlockMat);

  EXPECT_EQ(expMat._data, lowerTriaMat._data);
}

TEST(TriangularMatrix, setBlock_lowerBottomRight)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> lowerTriaMat(
    {{1, 0, 0}, 
     {1, 1, 0}, 
     {1, 1, 1}});
  const tracking::math::TriangularMatrix<float32, 2, true> lowerTriaBlockMat(
    {{2, 0},  
     {4, 3}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{1, 0, 0}, 
     {1, 2, 0}, 
     {1, 4, 3}});
  // clang-format on

  // call UUT
  lowerTriaMat.setBlock<2, 2, 0, 0, 1, 1>(lowerTriaBlockMat);

  EXPECT_EQ(expMat._data, lowerTriaMat._data);
}

TEST(TriangularMatrix, setBlock_upperTopLeft)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> upperTriaMat(
    {{1, 1, 1}, 
     {0, 1, 1}, 
     {0, 0, 1}});
  const tracking::math::TriangularMatrix<float32, 2, false> upperTriaBlockMat(
    {{2, 4},  
     {0, 3}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{2, 4, 1}, 
     {0, 3, 1}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  upperTriaMat.setBlock<2, 2, 0, 0, 0, 0>(upperTriaBlockMat);

  EXPECT_EQ(expMat._data, upperTriaMat._data);
}

TEST(TriangularMatrix, setBlock_upperTopRight)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> upperTriaMat(
    {{1, 1, 1}, 
     {0, 1, 1}, 
     {0, 0, 1}});
  const tracking::math::TriangularMatrix<float32, 2, false> upperTriaBlockMat(
    {{2, 4},  
     {0, 3}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 2, 4}, 
     {0, 1, 3}, 
     {0, 0, 1}});
  // clang-format on

  // call UUT
  upperTriaMat.setBlock<2, 2, 0, 0, 0, 1>(upperTriaBlockMat);

  EXPECT_EQ(expMat._data, upperTriaMat._data);
}

TEST(TriangularMatrix, setBlock_upperBottomRight)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> upperTriaMat(
    {{1, 1, 1}, 
     {0, 1, 1}, 
     {0, 0, 1}});
  const tracking::math::TriangularMatrix<float32, 2, false> upperTriaBlockMat(
    {{2, 4},  
     {0, 3}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, 1, 1}, 
     {0, 2, 4}, 
     {0, 0, 3}});
  // clang-format on

  // call UUT
  upperTriaMat.setBlock<2, 2, 0, 0, 1, 1>(upperTriaBlockMat);

  EXPECT_EQ(expMat._data, upperTriaMat._data);
}

TEST(TriangularMatrix, inverse_lower)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, true> lowerMat(
    {{1, 0, 0}, 
     {2, 4, 0}, 
     {3, 5, 6}});
  const tracking::math::TriangularMatrix<float32, 3, true> expMat(
    {{ 1.000000000000000,                  0,                 0},
     {-0.500000000000000,  0.250000000000000,                 0},
     {-0.083333333333333, -0.208333333333333, 0.166666666666667}});
  // clang-format on

  // call UUT
  //  auto invMat = lowerMat.inverse();

  //  EXPECT_EQ(expMat._data, invMat._data);
}

TEST(TriangularMatrix, inverse_upper)
{
  // clang-format off
  tracking::math::TriangularMatrix<float32, 3, false> upperMat(
    {{1, 2, 3}, 
     {0, 4, 5}, 
     {0, 0, 6}});
  const tracking::math::TriangularMatrix<float32, 3, false> expMat(
    {{1, -0.5,  -0.083333333333333},
     {0,  0.25, -0.208333333333333},
     {0,  0,     0.166666666666667}});
  // clang-format on

  // call UUT
  //  auto invMat = upperMat.inverse();

  //  EXPECT_EQ(expMat._data, invMat._data);
}
// NOLINTEND(modernize-use-trailing-return-type)
