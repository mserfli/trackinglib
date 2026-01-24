#ifndef D49F29DA_4806_421C_89F4_8462894DFBAA
#define D49F29DA_4806_421C_89F4_8462894DFBAA

#include "math/linalg/matrix_view.h"

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
inline MatrixView<ValueType_, Rows_, Cols_>::MatrixView(const Matrix<ValueType_, Rows_, Cols_>& matrix,
                                                        const sint32                            rowBegin,
                                                        const sint32                            colBegin,
                                                        const sint32                            rowEnd,
                                                        const sint32                            colEnd)
    : _matrix{matrix}
    , _rowBegin{rowBegin}
    , _colBegin{colBegin}
    , _rowCount{rowEnd - rowBegin + 1}
    , _colCount{colEnd - colBegin + 1}

{
  assert(_rowBegin >= 0);
  assert(_colBegin >= 0);
  assert(_rowCount <= Rows_);
  assert(_colCount <= Cols_);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
inline auto MatrixView<ValueType_, Rows_, Cols_>::operator()(sint32 row, sint32 col) const -> ValueType_
{
  assert(row < _rowCount);
  assert(col < _colCount);
  return _matrix.at_unsafe(_rowBegin + row, _colBegin + col);
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
inline auto MatrixView<ValueType_, Rows_, Cols_>::operator+(const Matrix<ValueType_, Rows_, Cols_>& other) const
    -> Matrix<ValueType_, Rows_, Cols_>
{
  auto result{other};
  for (auto row = 0; row < Rows_; ++row)
  {
    for (auto col = 0; col < Cols_; ++col)
    {
      result.at_unsafe(row, col) += this->operator()(row, col);
    }
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
inline auto MatrixView<ValueType_, Rows_, Cols_>::operator-(const Matrix<ValueType_, Rows_, Cols_>& other) const
    -> Matrix<ValueType_, Rows_, Cols_>
{
  auto result{other};
  for (auto row = 0; row < Rows_; ++row)
  {
    for (auto col = 0; col < Cols_; ++col)
    {
      result.at_unsafe(row, col) -= this->operator()(row, col);
    }
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
template <sint32 Cols2_>
inline auto MatrixView<ValueType_, Rows_, Cols_>::operator*(const Matrix<ValueType_, Cols_, Cols2_>& other) const
    -> Matrix<ValueType_, Rows_, Cols2_>
{
  Matrix<ValueType_, Rows_, Cols_> result;
  for (auto i = 0; i < Rows_; ++i)
  {
    for (auto k = 0; k < Cols_; ++k)
    {
      for (auto j = 0; j < Cols2_; ++j)
      {
        result.at_unsafe(i, j) += this->operator()(i, k) * other.at_unsafe(k, j);
      }
    }
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
inline auto MatrixView<ValueType_, Rows_, Cols_>::operator+(const ValueType_& other) const -> Matrix<ValueType_, Rows_, Cols_>
{
  Matrix<ValueType_, Rows_, Cols_> result;
  for (auto row = 0; row < _rowCount; ++row)
  {
    for (auto col = 0; col < _colCount; ++col)
    {
      result.at_unsafe(row, col) = this->operator()(row, col) + other;
    }
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
inline auto MatrixView<ValueType_, Rows_, Cols_>::operator-(const ValueType_& other) const -> Matrix<ValueType_, Rows_, Cols_>
{
  Matrix<ValueType_, Rows_, Cols_> result;
  for (auto row = 0; row < _rowCount; ++row)
  {
    for (auto col = 0; col < _colCount; ++col)
    {
      result.at_unsafe(row, col) = this->operator()(row, col) - other;
    }
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
inline auto MatrixView<ValueType_, Rows_, Cols_>::operator*(const ValueType_& other) const -> Matrix<ValueType_, Rows_, Cols_>
{
  Matrix<ValueType_, Rows_, Cols_> result;
  for (auto row = 0; row < _rowCount; ++row)
  {
    for (auto col = 0; col < _colCount; ++col)
    {
      result.at_unsafe(row, col) = this->operator()(row, col) * other;
    }
  }
  return result;
}

template <typename ValueType_, sint32 Rows_, sint32 Cols_>
inline auto MatrixView<ValueType_, Rows_, Cols_>::operator/(const ValueType_& other) const -> Matrix<ValueType_, Rows_, Cols_>
{
  Matrix<ValueType_, Rows_, Cols_> result;
  for (auto row = 0; row < _rowCount; ++row)
  {
    for (auto col = 0; col < _colCount; ++col)
    {
      result.at_unsafe(row, col) = this->operator()(row, col) / other;
    }
  }
  return result;
}

template <typename ValueType_, sint32 RowsA_, sint32 ColsA_, sint32 RowsB_, sint32 ColsB_>
auto operator+(const MatrixView<ValueType_, RowsA_, ColsA_>& a, const MatrixView<ValueType_, RowsB_, ColsB_>& b)
    -> Matrix<ValueType_, std::min(RowsA_, RowsB_), std::min(ColsA_, ColsB_)>
{
  assert(a.getRowCount() == b.getRowCount());
  assert(a.getColCount() == b.getColCount());
  Matrix<ValueType_, std::min(RowsA_, RowsB_), std::min(ColsA_, ColsB_)> result{};

  for (auto row = 0; row < a.getRowCount(); ++row)
  {
    for (auto col = 0; col < a.getColCount(); ++col)
    {
      result.at_unsafe(row, col) = a(row, col) + b(row, col);
    }
  }
  return result;
}

template <typename ValueType_, sint32 RowsA_, sint32 ColsA_, sint32 RowsB_, sint32 ColsB_>
auto operator-(const MatrixView<ValueType_, RowsA_, ColsA_>& a, const MatrixView<ValueType_, RowsB_, ColsB_>& b)
    -> Matrix<ValueType_, std::min(RowsA_, RowsB_), std::min(ColsA_, ColsB_)>
{
  assert(a.getRowCount() == b.getRowCount());
  assert(a.getColCount() == b.getColCount());
  Matrix<ValueType_, std::min(RowsA_, RowsB_), std::min(ColsA_, ColsB_)> result{};

  for (auto row = 0; row < a.getRowCount(); ++row)
  {
    for (auto col = 0; col < a.getColCount(); ++col)
    {
      result.at_unsafe(row, col) = a(row, col) - b(row, col);
    }
  }
  return result;
}

template <typename ValueType_, sint32 RowsA_, sint32 ColsA_, sint32 RowsB_, sint32 ColsB_>
auto operator*(const MatrixView<ValueType_, RowsA_, ColsA_>& a,
               const MatrixView<ValueType_, RowsB_, ColsB_>& b) -> Matrix<ValueType_, RowsA_, ColsB_>
{
  assert(a.getColCount() == b.getRowCount());
  Matrix<ValueType_, RowsA_, ColsB_> result{};

  for (auto i = 0; i < a.getRowCount(); ++i)
  {
    for (auto k = 0; k < a.getColCount(); ++k)
    {
      for (auto j = 0; j < b.getColCount(); ++j)
      {
        result.at_unsafe(i, j) += a(i, k) * b(k, j);
      }
    }
  }
  return result;
}

} // namespace math
} // namespace tracking

#endif // D49F29DA_4806_421C_89F4_8462894DFBAA
