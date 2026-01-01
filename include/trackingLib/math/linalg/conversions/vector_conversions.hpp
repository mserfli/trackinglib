#ifndef D350299D_28E6_4635_9D1B_F2C6D569A1C3
#define D350299D_28E6_4635_9D1B_F2C6D569A1C3

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

// VectorFromMatrixColumnView: Vector from MatrixColumnView
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

// VectorFromList: Vector from initializer_list<ValueType_>
template <typename ValueType_, sint32 Size_>
inline auto VectorFromList(const std::initializer_list<ValueType_>& list) -> Vector<ValueType_, Size_>
{
  assert(list.size() == Size_);

  Vector<ValueType_, Size_> tmp;
  auto                      iter = tmp.data().begin();
  std::copy(list.begin(), list.end(), iter);
  return tmp;
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

#endif // D350299D_28E6_4635_9D1B_F2C6D569A1C3
