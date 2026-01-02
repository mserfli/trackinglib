#ifndef C21D598C_CF10_4BAC_8857_B0DA4A653638
#define C21D598C_CF10_4BAC_8857_B0DA4A653638

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

// SquareFromList: SquareMatrix from initializer_list<initializer_list<ValueType_>>
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareFromList(const std::initializer_list<std::initializer_list<ValueType_>>& list)
    -> SquareMatrix<ValueType_, Size_, IsRowMajor_>
{
  SquareMatrix<ValueType_, Size_, IsRowMajor_> result{};

  // Validate row count
  if (list.size() != static_cast<std::size_t>(result.RowsInMem))
  {
    throw std::runtime_error("SquareFromList: expected " + std::to_string(result.RowsInMem) + " rows, got " +
                             std::to_string(list.size()));
  }

  // Validate column count for each row
  for (const auto& row : list)
  {
    if (row.size() != static_cast<std::size_t>(result.ColsInMem))
    {
      throw std::runtime_error("SquareFromList: expected " + std::to_string(result.ColsInMem) + " columns, got " +
                               std::to_string(row.size()));
    }
  }

  auto iter = result.data().begin();
  for (const auto& row : list)
  {
    std::copy(row.begin(), row.end(), iter);
    iter += row.size();
  }
  return result;
}

// SquareFromDiagonal: SquareMatrix from DiagonalMatrix
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
inline auto SquareFromDiagonal(const DiagonalMatrix<ValueType_, Size_>& diag) -> SquareMatrix<ValueType_, Size_, IsRowMajor_>
{
  SquareMatrix<ValueType_, Size_, IsRowMajor_> result{};
  for (sint32 i = 0; i < Size_; ++i)
  {
    result.at_unsafe(i, i) = diag.at_unsafe(i);
  }
  return result;
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // C21D598C_CF10_4BAC_8857_B0DA4A653638
