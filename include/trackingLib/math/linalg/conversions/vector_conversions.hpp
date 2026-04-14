#ifndef D350299D_28E6_4635_9D1B_F2C6D569A1C3
#define D350299D_28E6_4635_9D1B_F2C6D569A1C3

#include "conversions.h"
#include "math/linalg/matrix_column_view.hpp" // IWYU pragma: keep
#include "math/linalg/vector.hpp"             // IWYU pragma: keep

namespace tracking
{
namespace math
{
namespace conversions
{

/// \brief Creates a Vector from a MatrixColumnView
///
/// This function extracts the elements from a matrix column view to create a vector.
/// The column view must have the correct dimensions.
///
/// \tparam ValueType_ The atomic data type of internal elements
/// \tparam Size_ The size of the resulting vector
/// \param[in] colView The source matrix column view
/// \return Vector containing the elements from the column view
/// \note The column view row count must equal Size_, otherwise assertion fails
/// \see VectorFromList() for creating from initializer lists
/// \see MatrixFromVector() for the reverse conversion
template <typename ValueType_, sint32 Size_>
inline auto VectorFromMatrixColumnView(const MatrixColumnView<ValueType_, Size_, 1, true>& colView) -> Vector<ValueType_, Size_>
{
  assert(colView.getRowCount() == Size_);
  Vector<ValueType_, Size_> result;
  for (sint32 i = 0; i < Size_; ++i)
  {
    result.at_unsafe(i) = colView.at_unsafe(i);
  }
  return result;
}

} // namespace conversions
} // namespace math
} // namespace tracking

#endif // D350299D_28E6_4635_9D1B_F2C6D569A1C3
