#ifndef BB251A7C_F2DC_4075_B478_BA5931DF6CEC
#define BB251A7C_F2DC_4075_B478_BA5931DF6CEC

#include "math/linalg/modified_gram_schmidt.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
void ModifiedGramSchmidt<FloatType, Size>::run(TriangularMatrix<FloatType, Size, false>& u,
                                               DiagonalMatrix<FloatType, Size>&          d,
                                               const SquareMatrix<FloatType, Size>&      Phi,
                                               const bool                                transposeU)
{
  // M. S. Grewal and A. P. Andrews
  // Kalman Filtering: Theory and Practice Using MATLAB, 4th Edition
  // Wiley, 2014.
  //
  // Catherine Thornton's modified weighted Gram-Schmidt orthogonalization method

  // TODO(matthias): Grewal, p. 260 -> inplace product Phi*U
  auto PhiU = transposeU ? Phi * u.transpose() : Phi * u;
  auto Din  = d;
  u.setIdentity();
  FloatType sigma;
  for (sint32 i = Size - 1; i >= 0; --i)
  {
    sigma = static_cast<FloatType>(0.0);
    for (sint32 j = 0; j < Size; ++j)
    {
      sigma += (PhiU(i, j) * PhiU(i, j)) * Din[j];
    }
    d[i] = std::max(sigma, std::numeric_limits<FloatType>::epsilon());
    for (sint32 j = 0; j < i; ++j)
    {
      sigma = static_cast<FloatType>(0.0);
      for (sint32 k = 0; k < Size; ++k)
      {
        sigma += PhiU(i, k) * Din[k] * PhiU(j, k);
      }

      u(j, i) = sigma / d[i];

      for (sint32 k = 0; k < Size; ++k)
      {
        PhiU(j, k) -= u(j, i) * PhiU(i, k);
      }
    }
  }
}

template <typename FloatType, sint32 Size>
template <sint32 SizeQ>
void ModifiedGramSchmidt<FloatType, Size>::run(TriangularMatrix<FloatType, Size, false>& u,
                                               DiagonalMatrix<FloatType, Size>&          d,
                                               const SquareMatrix<FloatType, Size>&      Phi,
                                               const Matrix<FloatType, Size, SizeQ>&     G,
                                               const DiagonalMatrix<FloatType, SizeQ>&   Q)
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
  FloatType sigma;
  for (sint32 i = Size - 1; i >= 0; --i)
  {
    sigma = static_cast<FloatType>(0.0);
    for (sint32 j = 0; j < Size; ++j)
    {
      sigma += (PhiU(i, j) * PhiU(i, j)) * Din[j];
      if (j < SizeQ)
      {
        sigma += Gin(i, j) * Gin(i, j) * Q[j];
      }
    }
    d[i] = std::max(sigma, std::numeric_limits<FloatType>::epsilon());
    for (sint32 j = 0; j < i; ++j)
    {
      sigma = static_cast<FloatType>(0.0);
      for (sint32 k = 0; k < Size; ++k)
      {
        sigma += PhiU(i, k) * Din[k] * PhiU(j, k);
      }
      for (sint32 k = 0; k < SizeQ; ++k)
      {
        sigma += Gin(i, k) * Q[k] * Gin(j, k);
      }

      u(j, i) = sigma / d[i];

      for (sint32 k = 0; k < Size; ++k)
      {
        PhiU(j, k) -= u(j, i) * PhiU(i, k);
      }
      for (sint32 k = 0; k < SizeQ; ++k)
      {
        Gin(j, k) -= u(j, i) * Gin(i, k);
      }
    }
  }
}

} // namespace math
} // namespace tracking

#endif // BB251A7C_F2DC_4075_B478_BA5931DF6CEC
