#ifndef BB251A7C_F2DC_4075_B478_BA5931DF6CEC
#define BB251A7C_F2DC_4075_B478_BA5931DF6CEC

#include "math/linalg/modified_gram_schmidt.h"

#include "math/linalg/diagonal_matrix.hpp"   // IWYU pragma: keep
#include "math/linalg/matrix.hpp"            // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"     // IWYU pragma: keep
#include "math/linalg/triangular_matrix.hpp" // IWYU pragma: keep

namespace tracking
{
namespace math
{
template <typename ValueType_, sint32 Size_>
void ModifiedGramSchmidt<ValueType_, Size_>::run(TriangularMatrix<ValueType_, Size_, false, true>& u,
                                                 DiagonalMatrix<ValueType_, Size_>&                d,
                                                 const SquareMatrix<ValueType_, Size_, true>&      Phi)
{
  // M. S. Grewal and A. P. Andrews
  // Kalman Filtering: Theory and Practice Using MATLAB, 4th Edition
  // Wiley, 2014.
  //
  // Catherine Thornton's modified weighted Gram-Schmidt orthogonalization method
  // TODO(matthias): Grewal, p. 260 -> inplace product Phi*U
  // TODO(matthias): optimization - eliminate allocations in modified Gram-Schmidt for in-place updates
  // beneficial for small n without complexity overhead; add template-based loop unrolling for Size_ <= 10
  auto PhiU = Phi * u;
  auto Din  = d;
  u.setIdentity();
  ValueType_ sigma;
  for (sint32 i = Size_ - 1; i >= 0; --i)
  {
    sigma = static_cast<ValueType_>(0.0);
    for (sint32 j = 0; j < Size_; ++j)
    {
      sigma += (PhiU.at_unsafe(i, j) * PhiU.at_unsafe(i, j)) * Din.at_unsafe(j);
    }
    d.at_unsafe(i) = std::max(sigma, std::numeric_limits<ValueType_>::epsilon());
    for (sint32 j = 0; j < i; ++j)
    {
      sigma = static_cast<ValueType_>(0.0);
      for (sint32 k = 0; k < Size_; ++k)
      {
        sigma += PhiU.at_unsafe(i, k) * Din.at_unsafe(k) * PhiU.at_unsafe(j, k);
      }

      u.at_unsafe(j, i) = sigma / d.at_unsafe(i);

      for (sint32 k = 0; k < Size_; ++k)
      {
        PhiU.at_unsafe(j, k) -= u.at_unsafe(j, i) * PhiU.at_unsafe(i, k);
      }
    }
  }
}

template <typename ValueType_, sint32 Size_>
template <sint32 SizeQ_>
void ModifiedGramSchmidt<ValueType_, Size_>::run(TriangularMatrix<ValueType_, Size_, false, true>& u,
                                                 DiagonalMatrix<ValueType_, Size_>&                d,
                                                 const SquareMatrix<ValueType_, Size_, true>&      Phi,
                                                 const Matrix<ValueType_, Size_, SizeQ_, true>&    G,
                                                 const DiagonalMatrix<ValueType_, SizeQ_>&         Q)
{
  // M. S. Grewal and A. P. Andrews
  // Kalman Filtering: Theory and Practice Using MATLAB, 4th Edition
  // Wiley, 2014.
  //
  // Catherine Thornton's modified weighted Gram-Schmidt orthogonalization method
  // for the predictor update of the U-D factors of the covariance matrix
  // of estimation uncertainty in Kalman filtering

  // TODO(matthias): Grewal, p. 260 -> inplace product Phi*U
  // TODO(matthias): optimization - eliminate allocations in modified Gram-Schmidt for in-place updates
  // beneficial for small n without complexity overhead; add template-based loop unrolling for Size_ <= 10
  auto PhiU = Phi * u;
  auto Din  = d;
  auto Gin  = G;
  u.setIdentity();
  ValueType_ sigma;
  for (sint32 i = Size_ - 1; i >= 0; --i)
  {
    sigma = static_cast<ValueType_>(0.0);
    // process all state columns
    for (sint32 j = 0; j < Size_; ++j)
    {
      sigma += (PhiU.at_unsafe(i, j) * PhiU.at_unsafe(i, j)) * Din.at_unsafe(j);
    }
    // process all process noise columns
    for (sint32 j = 0; j < SizeQ_; ++j)
    {
      sigma += Gin.at_unsafe(i, j) * Gin.at_unsafe(i, j) * Q.at_unsafe(j);
    }
    d.at_unsafe(i) = std::max(sigma, std::numeric_limits<ValueType_>::epsilon());
    for (sint32 j = 0; j < i; ++j)
    {
      sigma = static_cast<ValueType_>(0.0);
      for (sint32 k = 0; k < Size_; ++k)
      {
        sigma += PhiU.at_unsafe(i, k) * Din.at_unsafe(k) * PhiU.at_unsafe(j, k);
      }
      for (sint32 k = 0; k < SizeQ_; ++k)
      {
        sigma += Gin.at_unsafe(i, k) * Q.at_unsafe(k) * Gin.at_unsafe(j, k);
      }

      u.at_unsafe(j, i) = sigma / d.at_unsafe(i);

      for (sint32 k = 0; k < Size_; ++k)
      {
        PhiU.at_unsafe(j, k) -= u.at_unsafe(j, i) * PhiU.at_unsafe(i, k);
      }
      for (sint32 k = 0; k < SizeQ_; ++k)
      {
        Gin.at_unsafe(j, k) -= u.at_unsafe(j, i) * Gin.at_unsafe(i, k);
      }
    }
  }
}

} // namespace math
} // namespace tracking

#endif // BB251A7C_F2DC_4075_B478_BA5931DF6CEC
