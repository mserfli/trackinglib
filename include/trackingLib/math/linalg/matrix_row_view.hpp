#ifndef E2428903_D53A_4EC7_89B5_8C772C073887
#define E2428903_D53A_4EC7_89B5_8C772C073887

#include "math/linalg/matrix_row_view.h"

#include "math/linalg/matrix.hpp"             // IWYU pragma: keep
#include "math/linalg/matrix_column_view.hpp" // IWYU pragma: keep
#include "math/linalg/vector.hpp"             // IWYU pragma: keep


namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
MatrixRowView<ValueType_, Rows_, Cols_, IsRowMajor_>::MatrixRowView(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& matrix,
                                                                    const sint32                                         row,
                                                                    const sint32                                         colBegin,
                                                                    const sint32                                         colEnd)
    : _matrix{matrix}
    , _row{row}
    , _colBegin{colBegin}
    , _colCount{colEnd - colBegin + 1}
{
  assert(_colBegin >= 0);
  assert(_colCount <= Cols_);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto MatrixRowView<ValueType_, Rows_, Cols_, IsRowMajor_>::at_unsafe(const sint32 idx) const -> ValueType_
{
  assert(idx < _colCount);
  const auto col = _colBegin + idx;
  return _matrix.at_unsafe(_row, col);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
inline auto MatrixRowView<ValueType_, Rows_, Cols_, IsRowMajor_>::operator*(
    const Matrix<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const -> Matrix<ValueType_, 1, Cols2_, IsRowMajor2_>
{
  static_assert(Rows2_ <= Cols_);
  assert(Rows2_ == _colCount);

  Matrix<ValueType_, 1, Cols2_, IsRowMajor2_> result{};
  for (auto col = 0; col < Cols2_; ++col)
  {
    for (auto row = 0; row < Rows2_; ++row)
    {
      result.at_unsafe(0, col) += at_unsafe(row) * other.at_unsafe(row, col);
    }
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
inline auto MatrixRowView<ValueType_, Rows_, Cols_, IsRowMajor_>::operator*(
    const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const -> ValueType_
{
  assert(_colCount == other.getRowCount());
  // calc dot product
  ValueType_ result{};
  for (auto idx = 0; idx < _colCount; ++idx)
  {
    result += at_unsafe(idx) * other.at_unsafe(idx);
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void MatrixRowView<ValueType_, Rows_, Cols_, IsRowMajor_>::print() const
{
  for (auto col = 0; col < _colCount; ++col)
  {
    std::cout << at_unsafe(col) << ", ";
  }
  std::cout << "\n" << std::endl;
}

} // namespace math
} // namespace tracking

#endif // E2428903_D53A_4EC7_89B5_8C772C073887
