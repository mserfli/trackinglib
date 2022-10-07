#ifndef B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
#define B84C34EE_5B17_49CF_8DC1_3BCF45A59A20

#include "math/linalg/agee_turner_rank1_update.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
inline void AgeeTurnerRank1Update<FloatType, Size>::run(TriangularMatrix<FloatType, Size, false>& u,
                                                        DiagonalMatrix<FloatType, Size>&          d,
                                                        const FloatType                           c,
                                                        const Vector<FloatType, Size>&            x,
                                                        const bool                                transposeU)
{
}


} // namespace math
} // namespace tracking

#endif // B84C34EE_5B17_49CF_8DC1_3BCF45A59A20
