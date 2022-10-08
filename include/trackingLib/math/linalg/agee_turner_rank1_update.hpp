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
                                                        const FloatType                           c,
                                                        const Vector<FloatType, Size>&            x,
                                                        const bool /*transposeU*/)
{
  // A summary on the UD Kalman Filter
  //   J. Humberto Ramos∗
  //   University of Florida, Shalimar, FL 32579
  //   Kevin M. Brink†
  //   U.S. Air Force Research Laboratory, Eglin Air Force Base, FL 32542
  //
  // https://arxiv.org/pdf/2203.06105.pdf

  assert(static_cast<FloatType>(0.0) < c && "Scalar c has to be positive");
  Vector<FloatType, Size> cIn{};
  cIn[Size - 1] = c;
  auto xIn      = x;
  for (auto j = Size - 1; j > 0; --j)
  {
    u(j, j)    = static_cast<FloatType>(1.0);
    cIn[j - 1] = cIn[j] * d[j]; // preset cIn[j - 1] as d[j] will be changed inplace next line
    d[j]       = std::max(d[j] + cIn[j] * pow<2>(xIn[j]), std::numeric_limits<FloatType>::epsilon());

    for (auto k = 0; k < j; ++k)
    {
      xIn[k] -= xIn[j] * u(k, j);
      u(k, j) += cIn[j] * xIn[j] * xIn[k] / d[j];
    }
    cIn[j - 1] /= d[j];
  }
  d[0] += cIn[0] * pow<2>(xIn[0]);
}

} // namespace math
} // namespace tracking

#endif // B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
