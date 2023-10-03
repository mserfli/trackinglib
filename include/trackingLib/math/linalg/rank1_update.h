#ifndef E23E9AC4_E199_4264_B0DB_0DE4B42F3447
#define E23E9AC4_E199_4264_B0DB_0DE4B42F3447

#include "base/first_include.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/triangular_matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size>
class Rank1Update
{
public:
  static void run(TriangularMatrix<FloatType, Size, false>& u,
                  DiagonalMatrix<FloatType, Size>&          d,
                  FloatType                                 c, // we need a copy internally
                  Vector<FloatType, Size>                   x);                  // we need a copy internally

  static void run(TriangularMatrix<FloatType, Size, true>& l,
                  DiagonalMatrix<FloatType, Size>&         d,
                  FloatType                                c, // we need a copy internally
                  Vector<FloatType, Size>                  x);                 // we need a copy internally
};

} // namespace math
} // namespace tracking

#endif // E23E9AC4_E199_4264_B0DB_0DE4B42F3447
