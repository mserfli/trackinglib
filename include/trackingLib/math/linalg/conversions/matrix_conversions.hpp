#ifndef B83EA5A9_DE89_46DE_A570_E875E0EEFC5C
#define B83EA5A9_DE89_46DE_A570_E875E0EEFC5C

#include "conversions.h"
#include "math/linalg/matrix_column_view.hpp" // IWYU pragma: keep
#include "math/linalg/vector.hpp"             // IWYU pragma: keep

namespace tracking
{
namespace math
{
namespace conversions
{

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
