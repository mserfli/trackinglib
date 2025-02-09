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

template <typename FloatType_, sint32 Size_, bool IsRowMajor_>
class Rank1Update
{
public:
  static void run(TriangularMatrix<FloatType_, Size_, false, IsRowMajor_>& u,
                  DiagonalMatrix<FloatType_, Size_>&                       d,
                  FloatType_                                               c, // we need a copy internally
                  Vector<FloatType_, Size_>                                x);                               // we need a copy internally

  static void run(TriangularMatrix<FloatType_, Size_, true, IsRowMajor_>& l,
                  DiagonalMatrix<FloatType_, Size_>&                      d,
                  FloatType_                                              c, // we need a copy internally
                  Vector<FloatType_, Size_>                               x);                              // we need a copy internally
};

} // namespace math
} // namespace tracking

#endif // E23E9AC4_E199_4264_B0DB_0DE4B42F3447
