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

/// \brief Creates a SquareMatrix from a nested initializer list
///
/// This function constructs a SquareMatrix from a nested initializer list where each inner list
/// represents a row of the matrix. The dimensions must be square and match the template parameter.
///
/// \tparam ValueType_ The value type of matrix elements
/// \tparam Size_ The dimension of the square matrix
/// \tparam IsRowMajor_ The storage layout (true for row-major, false for column-major)
/// \param[in] list Nested initializer list where outer list contains rows and inner lists contain values
/// \return SquareMatrix instance initialized with the provided values
/// \throws std::runtime_error If the list dimensions don't match the square matrix size
/// \see SquareFromDiagonal() for creating from diagonal matrices
/// \see MatrixFromList() for general matrix creation
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

/// \brief Creates a SquareMatrix from a DiagonalMatrix
///
/// This function creates a square matrix with the diagonal elements from a diagonal matrix.
/// All off-diagonal elements are set to zero.
///
/// \tparam ValueType_ The value type of matrix elements
/// \tparam Size_ The dimension of the matrices
/// \tparam IsRowMajor_ The storage layout of the resulting square matrix
/// \param[in] diag The source diagonal matrix
/// \return SquareMatrix with diagonal elements from the input and zeros elsewhere
/// \see DiagonalFromSquare() for the reverse conversion
/// \see SquareFromList() for creating from initializer lists
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
