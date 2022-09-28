#include "gtest/gtest.h"
#include <trackingLib/math/linalg/triangular_matrix.h>

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::math::TriangularMatrix<float32, 3, true>; 
template class tracking::math::TriangularMatrix<float32, 3, false>; 
template auto  tracking::math::TriangularMatrix<float32, 3, false>::solve<1>(const tracking::math::Matrix<float32, 3, 1>&) const -> Matrix<float32, 3, 1>;
template void  tracking::math::TriangularMatrix<float32, 3, false>::setBlock<2, 2, 0, 0, 0, 0>(const tracking::math::TriangularMatrix<float32, 2, false>&);

TEST(TriangularMatrix, setBlock_lowerTopLeft) 
{
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
    
    // call UUT
    lowerTriaMat.setBlock<2, 2, 0, 0, 0, 0>(lowerTriaBlockMat);

    EXPECT_EQ(expMat._data, lowerTriaMat._data);
}

TEST(TriangularMatrix, setBlock_lowerBottomLeft) 
{
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
    
    // call UUT
    lowerTriaMat.setBlock<2, 2, 0, 0, 1, 0>(lowerTriaBlockMat);

    EXPECT_EQ(expMat._data, lowerTriaMat._data);
}

TEST(TriangularMatrix, setBlock_lowerBottomRight) 
{
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
    
    // call UUT
    lowerTriaMat.setBlock<2, 2, 0, 0, 1, 1>(lowerTriaBlockMat);

    EXPECT_EQ(expMat._data, lowerTriaMat._data);
}

TEST(TriangularMatrix, setBlock_upperTopLeft) 
{
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
    
    // call UUT
    upperTriaMat.setBlock<2, 2, 0, 0, 0, 0>(upperTriaBlockMat);

    EXPECT_EQ(expMat._data, upperTriaMat._data);
}

TEST(TriangularMatrix, setBlock_upperTopRight) 
{
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
    
    // call UUT
    upperTriaMat.setBlock<2, 2, 0, 0, 0, 1>(upperTriaBlockMat);

    EXPECT_EQ(expMat._data, upperTriaMat._data);
}

TEST(TriangularMatrix, setBlock_upperBottomRight) 
{
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
    
    // call UUT
    upperTriaMat.setBlock<2, 2, 0, 0, 1, 1>(upperTriaBlockMat);

    EXPECT_EQ(expMat._data, upperTriaMat._data);
}
// NOLINTEND(modernize-use-trailing-return-type)
