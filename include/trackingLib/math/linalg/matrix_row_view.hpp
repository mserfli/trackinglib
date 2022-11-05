#ifndef E2428903_D53A_4EC7_89B5_8C772C073887
#define E2428903_D53A_4EC7_89B5_8C772C073887

#include "math/linalg/matrix_row_view.h"

#include "math/linalg/matrix_column_view.hpp"

template <typename FloatType, sint32 Rows, sint32 Cols>
tracking::math::MatrixRowView<FloatType, Rows, Cols>::MatrixRowView(const Matrix<FloatType, Rows, Cols>& matrix,
                                                                    const sint32                         row,
                                                                    const sint32                         colBegin,
                                                                    const sint32                         colEnd)
    : _matrix{matrix}
    , _row{row}
    , _colBegin{colBegin}
    , _colCount{colEnd-colBegin+1}
{
  assert(_colBegin >= 0);
  assert(_colCount <= Cols);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto tracking::math::MatrixRowView<FloatType, Rows, Cols>::operator[](const sint32 idx) const -> FloatType
{
  assert(idx < _colCount);
  const auto col = _colBegin + idx;
  return _matrix(_row, col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Rows2, sint32 Cols2>
inline auto tracking::math::MatrixRowView<FloatType, Rows, Cols>::operator*(const Matrix<FloatType, Rows2, Cols2>& other) const
    -> Matrix<FloatType, 1, Cols2>
{
  static_assert(Rows2 <= Cols);
  assert(Rows2 == _colCount);

  Matrix<FloatType, 1, Cols2> result{};
  for (auto col = 0; col < Cols2; ++col)
  {
    for (auto row = 0; row < Rows2; ++row)
    {
      result(0, col) += this->operator[](row) * other(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Rows2, sint32 Cols2>
inline auto tracking::math::MatrixRowView<FloatType, Rows, Cols>::operator*(
    const MatrixColumnView<FloatType, Rows2, Cols2>& other) const -> FloatType
{
  assert(_colCount == other.getRowCount());
  // calc dot product
  FloatType result{};
  for (auto idx = 0; idx < _colCount; ++idx)
  {
    result += this->operator[](idx) * other[idx];
  }
  return result;
}

#endif // E2428903_D53A_4EC7_89B5_8C772C073887
