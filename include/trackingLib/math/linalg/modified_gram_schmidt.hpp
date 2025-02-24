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
template <typename FloatType_, sint32 Size_>
template <bool IsRegular_>
void ModifiedGramSchmidt<FloatType_, Size_>::run(TriangularMatrix<FloatType_, Size_, !IsRegular_, IsRegular_>& u,
                                                 DiagonalMatrix<FloatType_, Size_>&                            d,
                                                 SquareMatrix<FloatType_, Size_, IsRegular_>&                  PhiU)
{
  auto Din = d;
  u.setIdentity();
  FloatType_ sigma;
  if constexpr (IsRegular_)
  {
    for (sint32 i = Size_ - 1; i >= 0; --i)
    {
      sigma = static_cast<FloatType_>(0.0);
      for (sint32 j = 0; j < Size_; ++j)
      {
        sigma += (PhiU.at_unsafe(i, j) * PhiU.at_unsafe(i, j)) * Din.at_unsafe(j);
      }
      d.at_unsafe(i) = std::max(sigma, std::numeric_limits<FloatType_>::epsilon());
      for (sint32 j = 0; j < i; ++j)
      {
        sigma = static_cast<FloatType_>(0.0);
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
  else
  {
    // fill this section with the code from the other branch
  }
}

template <typename FloatType_, sint32 Size_>
void ModifiedGramSchmidt<FloatType_, Size_>::run(TriangularMatrix<FloatType_, Size_, false, true>& u,
                                                 DiagonalMatrix<FloatType_, Size_>&                d,
                                                 const SquareMatrix<FloatType_, Size_, true>&      Phi,
                                                 const bool                                        isInverse)
{
  // M. S. Grewal and A. P. Andrews
  // Kalman Filtering: Theory and Practice Using MATLAB, 4th Edition
  // Wiley, 2014.
  //
  // Catherine Thornton's modified weighted Gram-Schmidt orthogonalization method
  // isInverse = false: Phi*UDU'*Phi'
  // isInverse = true: inv(Phi)'*U'DU*inv(Phi)

  // TODO(matthias): Grewal, p. 260 -> inplace product Phi*U
  if (isInverse)
  {
    auto PhiU = SquareMatrix<FloatType_, Size_, false>{Phi.inverse() * u.transpose()};
    run<false>(u.transpose(), d, PhiU);
  }
  else
  {
    auto PhiU = SquareMatrix<FloatType_, Size_, true>{Phi * u};
    run<true>(u, d, PhiU);
  }
}

template <typename FloatType_, sint32 Size_>
template <sint32 SizeQ_>
void ModifiedGramSchmidt<FloatType_, Size_>::run(TriangularMatrix<FloatType_, Size_, false, true>& u,
                                                 DiagonalMatrix<FloatType_, Size_>&                d,
                                                 const SquareMatrix<FloatType_, Size_, true>&      Phi,
                                                 const Matrix<FloatType_, Size_, SizeQ_, true>&    G,
                                                 const DiagonalMatrix<FloatType_, SizeQ_>&         Q)
{
  // M. S. Grewal and A. P. Andrews
  // Kalman Filtering: Theory and Practice Using MATLAB, 4th Edition
  // Wiley, 2014.
  //
  // Catherine Thornton's modified weighted Gram-Schmidt orthogonalization method
  // for the predictor update of the U-D factors of the covariance matrix
  // of estimation uncertainty in Kalman filtering

  // TODO(matthias): Grewal, p. 260 -> inplace product Phi*U
  auto PhiU = Phi * u;
  auto Din  = d;
  auto Gin  = G;
  u.setIdentity();
  FloatType_ sigma;
  for (sint32 i = Size_ - 1; i >= 0; --i)
  {
    sigma = static_cast<FloatType_>(0.0);
    for (sint32 j = 0; j < Size_; ++j)
    {
      sigma += (PhiU.at_unsafe(i, j) * PhiU.at_unsafe(i, j)) * Din.at_unsafe(j);
      if (j < SizeQ_)
      {
        sigma += Gin.at_unsafe(i, j) * Gin.at_unsafe(i, j) * Q.at_unsafe(j);
      }
    }
    d.at_unsafe(i) = std::max(sigma, std::numeric_limits<FloatType_>::epsilon());
    for (sint32 j = 0; j < i; ++j)
    {
      sigma = static_cast<FloatType_>(0.0);
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
