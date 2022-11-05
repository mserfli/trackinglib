#ifndef DAD67FEF_0E47_455C_AC3E_CE31BDC7EE3F
#define DAD67FEF_0E47_455C_AC3E_CE31BDC7EE3F

#include "math/linalg/matrix_column_view.h"

template <typename FloatType, sint32 Rows, sint32 Cols>
inline tracking::math::MatrixColumnView<FloatType, Rows, Cols>::MatrixColumnView(const Matrix<FloatType, Rows, Cols>& matrix,
                                                                                 const sint32                         col,
                                                                                 const sint32                         rowBegin,
                                                                                 const sint32                         rowEnd)
    : _matrix{matrix}
    , _col{col}
    , _rowBegin{rowBegin}
    , _rowCount{rowEnd-rowBegin+1}
{
  assert(_rowBegin >= 0);
  assert(_rowCount <= Rows);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto tracking::math::MatrixColumnView<FloatType, Rows, Cols>::operator[](const sint32 idx) const -> FloatType
{
  assert(idx < _rowCount);
  const auto row = _rowBegin + idx;
  return _matrix(row, _col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Rows2>
inline auto tracking::math::MatrixColumnView<FloatType, Rows, Cols>::operator*(const Vector<FloatType, Rows2>& other) const
    -> FloatType
{
  static_assert(Rows2 <= Rows);
  assert(Rows2 == _rowCount);
  // calc dot product
  FloatType result{};
  for (auto row = 0; row < Rows2; ++row)
  {
    result += this->operator[](row) * other[row];
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Rows2, sint32 Cols2>
inline auto tracking::math::MatrixColumnView<FloatType, Rows, Cols>::operator*(const MatrixColumnView<FloatType, Rows2, Cols2>& other) const -> FloatType
{
  assert(other.getRowCount() == _rowCount);
  // calc dot product
  FloatType result{};
  for (auto row = 0; row < _rowCount; ++row)
  {
    result += this->operator[](row) * other[row];
  }
  return result;

}

template <typename FloatType, sint32 Rows, sint32 Cols, sint32 Rows2, sint32 Cols2>
inline auto tracking::math::operator*(const Matrix<FloatType, Rows, Cols>&             mat,
                                      const MatrixColumnView<FloatType, Rows2, Cols2>& colView) -> Vector<FloatType, Rows>
{
  static_assert(Cols <= Rows2);
  assert(Cols == colView.getRowCount());

  Vector<FloatType, Rows> result{};
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result[row] += mat(row, col) * colView[col];
    }
  }
  return result;
}

#endif // DAD67FEF_0E47_455C_AC3E_CE31BDC7EE3F
