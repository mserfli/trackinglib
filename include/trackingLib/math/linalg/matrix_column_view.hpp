#ifndef DAD67FEF_0E47_455C_AC3E_CE31BDC7EE3F
#define DAD67FEF_0E47_455C_AC3E_CE31BDC7EE3F

#include "math/linalg/matrix_column_view.h"

namespace tracking::math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline MatrixColumnView<ValueType_, Rows_, Cols_, IsRowMajor_>::MatrixColumnView(
    const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& matrix, const sint32 col, const sint32 rowBegin, const sint32 rowEnd)
    : _matrix{matrix}
    , _col{col}
    , _rowBegin{rowBegin}
    , _rowCount{rowEnd - rowBegin + 1}
{
  assert(_rowBegin >= 0);
  assert(_rowCount <= Rows_);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline auto MatrixColumnView<ValueType_, Rows_, Cols_, IsRowMajor_>::at_unsafe(const sint32 idx) const -> ValueType_
{
  assert(idx < _rowCount);
  const auto row = _rowBegin + idx;
  return _matrix.at_unsafe(row, _col);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <sint32 Rows2_>
inline auto MatrixColumnView<ValueType_, Rows_, Cols_, IsRowMajor_>::operator*(const Vector<ValueType_, Rows2_>& other) const
    -> ValueType_
{
  static_assert(Rows2_ <= Rows_);
  assert(Rows2_ == _rowCount);
  // calc dot product
  ValueType_ result{};
  for (auto row = 0; row < Rows2_; ++row)
  {
    result += this->at_unsafe(row) * other.at_unsafe(row);
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
template <sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
inline auto MatrixColumnView<ValueType_, Rows_, Cols_, IsRowMajor_>::operator*(
    const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& other) const -> ValueType_
{
  assert(other.getRowCount() == _rowCount);
  // calc dot product
  ValueType_ result{};
  for (auto row = 0; row < _rowCount; ++row)
  {
    result += this->at_unsafe(row) * other.at_unsafe(row);
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
inline void MatrixColumnView<ValueType_, Rows_, Cols_, IsRowMajor_>::print() const
{
  for (auto row = 0; row < _rowCount; ++row)
  {
    std::cout << at_unsafe(row) << ", ";
  }
  std::cout << "\n" << std::endl;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_, sint32 Rows2_, sint32 Cols2_, bool IsRowMajor2_>
inline auto operator*(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& mat,
                      const MatrixColumnView<ValueType_, Rows2_, Cols2_, IsRowMajor2_>& colView) -> Vector<ValueType_, Rows_>
{
  static_assert(Cols_ <= Rows2_);
  assert(Cols_ == colView.getRowCount());

  Vector<ValueType_, Rows_> result{};
  for (auto row = 0; row < Rows_; ++row)
  {
    for (auto col = 0; col < Cols_; ++col)
    {
      result.at_unsafe(row) += mat.at_unsafe(row, col) * colView.at_unsafe(col);
    }
  }
  return result;
}

} // namespace tracking::math

#endif // DAD67FEF_0E47_455C_AC3E_CE31BDC7EE3F
