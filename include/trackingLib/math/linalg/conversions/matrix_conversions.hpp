#ifndef B83EA5A9_DE89_46DE_A570_E875E0EEFC5C
#define B83EA5A9_DE89_46DE_A570_E875E0EEFC5C

#include "conversions.h"
#include "math/linalg/matrix_column_view.hpp" // IWYU pragma: keep
#include "math/linalg/vector.hpp"             // IWYU pragma: keep
#include <initializer_list>

namespace tracking
{
namespace math
{
namespace conversions
{

// MatrixFromList: Matrix from initializer_list<initializer_list<ValueType_>>
template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto MatrixFromList(const std::initializer_list<std::initializer_list<ValueType_>>& list)
    -> Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>
{
  Matrix<ValueType_, Rows_, Cols_, IsRowMajor_> result{};

  // Validate row count
  if (list.size() != static_cast<std::size_t>(result.RowsInMem))
  {
    throw std::runtime_error("MatrixFromList: expected " + std::to_string(result.RowsInMem) + " rows, got " +
                             std::to_string(list.size()));
  }

  // Validate column count for each row
  for (const auto& row : list)
  {
    if (row.size() != static_cast<std::size_t>(result.ColsInMem))
    {
      throw std::runtime_error("MatrixFromList: expected " + std::to_string(result.ColsInMem) + " columns, got " +
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

// MatrixFromVector: Matrix from Vector
template <typename ValueType_, sint32 Size_>
inline auto MatrixFromVector(const Vector<ValueType_, Size_>& vec) -> Matrix<ValueType_, Size_, 1, true>
{
  Matrix<ValueType_, Size_, 1, true> result{};
  for (sint32 i = 0; i < Size_; ++i)
  {
    result.at_unsafe(i, 0) = vec.at_unsafe(i);
  }
  return result;
}


} // namespace conversions
} // namespace math
} // namespace tracking

#endif // B83EA5A9_DE89_46DE_A570_E875E0EEFC5C
