#include "gtest/gtest.h"
#include <trackingLib/math/linalg/diagonal_matrix.h>

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::math::DiagonalMatrix<float32,3>;
template void tracking::math::DiagonalMatrix<float32,3>::setBlock<2,2,0,0>(const DiagonalMatrix<float32,2>&); 

TEST(DiagonalMatrix, setBlock_topLeft) 
{
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
    
    // call UUT
    diagMat.setBlock<2, 2, 0, 0>(diagBlockMat);

    EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, setBlock_bottomRight) 
{
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
    
    // call UUT
    diagMat.setBlock<2, 2, 0, 1>(diagBlockMat);

    EXPECT_EQ(expMat._data, diagMat._data);
}

TEST(DiagonalMatrix, inverse)
{
    const tracking::math::DiagonalMatrix<float32, 3> diagMat(
      {{1, 0, 0}, 
       {0, 2, 0}, 
       {0, 0, 4}});
    const tracking::math::DiagonalMatrix<float32, 3> expMat(
      {{1, 0,   0}, 
       {0, 0.5, 0}, 
       {0, 0,   0.25}});

    auto invMat = diagMat.inverse();

    EXPECT_EQ(expMat._data, invMat._data);
}

TEST(DiagonalMatrix, inverse_inplace)
{
    tracking::math::DiagonalMatrix<float32, 3> diagMat(
      {{1, 0, 0}, 
       {0, 2, 0}, 
       {0, 0, 4}});
    const tracking::math::DiagonalMatrix<float32, 3> expMat(
      {{1, 0,   0}, 
       {0, 0.5, 0}, 
       {0, 0,   0.25}});

    diagMat.inverse();

    EXPECT_EQ(expMat._data, diagMat._data);
}
// NOLINTEND(modernize-use-trailing-return-type)
