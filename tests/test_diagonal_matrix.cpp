#include "gtest/gtest.h"
#include <trackingLib/math/linalg/diagonal_matrix.h>

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::math::DiagonalMatrix<float32, 3>;
template void tracking::math::DiagonalMatrix<float32, 3>::setBlock<2, 2, 0, 0>(const DiagonalMatrix<float32, 2>&);

TEST(DiagonalMatrix, setBlock_topLeft)
{
    // clang-format off
    tracking::math::DiagonalMatrix<float32, 3> diagMat(
      {{1, 0, 0}, 
       {0, 1, 0}, 
       {0, 0, 1}});
    const tracking::math::DiagonalMatrix<float32, 2> diagBlockMat(
      {{2, 0},  
       {0, 3}});
    const tracking::math::DiagonalMatrix<float32, 3> expMat(
      {{2, 0, 0}, 
       {0, 3, 0}, 
       {0, 0, 1}});
  // clang-format on
  
  // call UUT
  diagMat.setBlock<2, 2, 0, 0>(diagBlockMat);

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, setBlock_bottomRight)
{
    // clang-format off
    tracking::math::DiagonalMatrix<float32, 3> diagMat(
      {{1, 0, 0}, 
       {0, 1, 0}, 
       {0, 0, 1}});
    const tracking::math::DiagonalMatrix<float32, 2> diagBlockMat(
      {{2, 0},  
       {0, 3}});
    const tracking::math::DiagonalMatrix<float32, 3> expMat(
      {{1, 0, 0}, 
       {0, 2, 0}, 
       {0, 0, 3}});
  // clang-format on
  
  // call UUT
  diagMat.setBlock<2, 2, 0, 1>(diagBlockMat);

  EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, inverse)
{
    // clang-format off
    const tracking::math::DiagonalMatrix<float32, 3> diagMat(
      {{1, 0, 0}, 
       {0, 2, 0}, 
       {0, 0, 4}});
    const tracking::math::DiagonalMatrix<float32, 3> expMat(
      {{1, 0,   0}, 
       {0, 0.5, 0}, 
       {0, 0,   0.25}});
  // clang-format on

  // call UUT
  auto invMat = diagMat.inverse();

  EXPECT_EQ(expMat._data, invMat._data);
}

TEST(DiagonalMatrix, inverse_inplace)
{
    // clang-format off
    tracking::math::DiagonalMatrix<float32, 3> diagMat(
      {{1, 0, 0}, 
       {0, 2, 0}, 
       {0, 0, 4}});
    const tracking::math::DiagonalMatrix<float32, 3> expMat(
      {{1, 0,   0}, 
       {0, 0.5, 0}, 
       {0, 0,   0.25}});
  // clang-format on

  // call UUT
  diagMat.inverse();

  EXPECT_EQ(expMat._data, diagMat._data);
}
// NOLINTEND(modernize-use-trailing-return-type)
