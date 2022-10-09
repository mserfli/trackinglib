#ifndef B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
#define B84C34EE_5B17_49CF_8DC1_3BCF45A59A20

#include "math/linalg/agee_turner_rank1_update.h"

#include "math/analysis/functions.h"
#include "math/linalg/vector.h"
#include <limits>

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
inline void AgeeTurnerRank1Update<FloatType, Size>::run(TriangularMatrix<FloatType, Size, false>& u,
                                                        DiagonalMatrix<FloatType, Size>&          d,
                                                        FloatType                                 c,
                                                        Vector<FloatType, Size>                   x)
{
  // Navigation Filter Best Practices
  //   Carpenter, Russell J. and D'Souza, Christopher N.
  //   NASA
  // page 89

  FloatType beta{};
  FloatType eta{};
  FloatType dj{};
  for (auto j = Size - 1; j > 0; --j)
  {
    dj   = d[j];
    d[j] = std::max(dj + (c * pow<2>(x[j])), std::numeric_limits<FloatType>::epsilon());
    u(j, j)    = static_cast<FloatType>(1.0);

    beta = c / d[j]; // division is safe because of std::max before
    eta = beta * x[j];
    for (auto i = 0; i < j; ++i)
    {
      x[i] -= u(i, j) * x[j];
      u(i, j) += x[i] * eta;
    }
    c = beta * dj; // use here the backup value if d[j] stored at the beginning
  }
  d[0] += c * pow<2>(x[0]);
}

} // namespace math
} // namespace tracking

#endif // B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
