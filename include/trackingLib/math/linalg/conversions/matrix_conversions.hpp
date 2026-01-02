#ifndef B83EA5A9_DE89_46DE_A570_E875E0EEFC5C
#define B83EA5A9_DE89_46DE_A570_E875E0EEFC5C

#include "conversions.h"
#include "math/linalg/matrix_column_view.hpp" // IWYU pragma: keep
#include "math/linalg/vector.hpp"             // IWYU pragma: keep
#include <initializer_list>
#include <stdexcept>

namespace tracking
{
namespace math
{
namespace conversions
{

/// \brief Creates a Matrix from a nested initializer list
///
/// This function constructs a Matrix from a nested initializer list where each inner list
/// represents a row of the matrix. The dimensions must match the template parameters exactly.
///
/// \tparam ValueType_ The value type of matrix elements (e.g., float32, float64)
/// \tparam Rows_ The number of rows in the resulting matrix
/// \tparam Cols_ The number of columns in the resulting matrix
/// \tparam IsRowMajor_ The storage layout (true for row-major, false for column-major)
/// \param[in] list Nested initializer list where outer list contains rows and inner lists contain column values
/// \return Matrix instance initialized with the provided values
/// \throws std::runtime_error If the list dimensions don't match the matrix dimensions
/// \see MatrixFromVector() for creating matrices from vectors
/// \see SquareFromList() for square matrix creation
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

/// \brief Creates a single-column Matrix from a Vector
///
/// This function converts a Vector into a Matrix with one column, effectively
/// representing the vector as a column matrix. The resulting matrix uses row-major storage.
///
/// \tparam ValueType_ The value type of vector and matrix elements
/// \tparam Size_ The size of the vector and number of rows in the matrix
/// \param[in] vec The source vector to convert
/// \return Matrix with Size_ rows and 1 column containing the vector elements
/// \see VectorFromMatrixColumnView() for the reverse conversion
/// \see MatrixFromList() for creating matrices from initializer lists
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
