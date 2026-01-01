#ifndef A000E27D_C91E_4768_A8FD_B292AB7B986A
#define A000E27D_C91E_4768_A8FD_B292AB7B986A

#include "conversions.h"
#include "math/linalg/diagonal_matrix.hpp" // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"   // IWYU pragma: keep
#include <initializer_list>

namespace tracking
{
namespace math
{
namespace conversions
{

// DiagonalFromSquare: DiagonalMatrix from SquareMatrix
// <target>From<source> pattern
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto DiagonalFromSquare(const SquareMatrix<ValueType_, Size_, IsRowMajor_>& mat) -> DiagonalMatrix<ValueType_, Size_>
{
  DiagonalMatrix<ValueType_, Size_> result;
  for (sint32 i = 0; i < Size_; ++i)
  {
    result.at_unsafe(i) = mat.at_unsafe(i, i);
  }
  return result;
}

// DiagonalFromList: DiagonalMatrix from initializer_list<ValueType_>
// OPTIMIZED: Overloaded function for different list types
template <typename ValueType_, sint32 Size_>
inline auto DiagonalFromList(const std::initializer_list<ValueType_>& list) -> DiagonalMatrix<ValueType_, Size_>
{
  assert((list.size() == Size_) && "Mismatching size of intializer list");

  DiagonalMatrix<ValueType_, Size_> diag{};
  // fill diagonal elements
  sint32 idx = 0;
  for (auto val : list)
  {
    diag.at_unsafe(idx++) = val;
  }
  return diag;
}

// DiagonalFromList: DiagonalMatrix from initializer_list<initializer_list<ValueType_>>
// OPTIMIZED: Overloaded function for nested list
template <typename ValueType_, sint32 Size_>
inline auto DiagonalFromList(const std::initializer_list<std::initializer_list<ValueType_>>& list)
    -> DiagonalMatrix<ValueType_, Size_>
{
  assert(list.size() == Size_);
  assert(list.begin()->size() == Size_);

  DiagonalMatrix<ValueType_, Size_> diag{};
  // copy diagonal elements from list
  sint32 idx = 0;
  for (const auto& rowList : list)
  {
    assert((rowList.size() == Size_) && "Mismatching size of intializer list");
    diag.at_unsafe(idx) = *(rowList.begin() + idx);
    ++idx;
  }
  return diag;
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // A000E27D_C91E_4768_A8FD_B292AB7B986A
