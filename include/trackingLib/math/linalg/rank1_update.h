#ifndef E23E9AC4_E199_4264_B0DB_0DE4B42F3447
#define E23E9AC4_E199_4264_B0DB_0DE4B42F3447

#include "base/first_include.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class Vector; // LCOV_EXCL_LINE

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
