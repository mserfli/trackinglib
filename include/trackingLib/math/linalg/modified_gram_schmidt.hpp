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
                                               const SquareMatrix<FloatType, Size>&      phi,
                                               const bool transposeU)
{
  // M. S. Grewal and A. P. Andrews
  // Kalman Filtering: Theory and Practice Using MATLAB, 4th Edition
  // Wiley, 2014.
  //
  // Catherine Thornton's modified weighted Gram-Schmidt orthogonalization method

  auto PhiU = transposeU ? phi * u.transpose() : phi * u;
  auto Din  = d;
  u.setIdentity();
  FloatType sigma;
  for (sint32 i = Size - 1; i >= 0; --i)
  {
    sigma = static_cast<FloatType>(0.0);
    for (sint32 j = 0; j < Size; ++j)
    {
      sigma += (PhiU(i, j) * PhiU(i, j)) * Din(j, j);
    }
    d(i, i) = std::max(sigma, std::numeric_limits<FloatType>::epsilon());
    for (sint32 j = 0; j < i; ++j)
    {
      sigma = static_cast<FloatType>(0.0);
      for (sint32 k = 0; k < Size; ++k)
      {
        sigma += PhiU(i, k) * Din(k, k) * PhiU(j, k);
      }

      u(j, i) = sigma / d(i, i);

      for (sint32 k = 0; k < Size; ++k)
      {
        PhiU(j, k) -= u(j, i) * PhiU(i, k);
      }
    }
  }
}

} // namespace math
} // namespace tracking

#endif // BB251A7C_F2DC_4075_B478_BA5931DF6CEC
