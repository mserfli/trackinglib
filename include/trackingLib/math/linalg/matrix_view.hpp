#ifndef D49F29DA_4806_421C_89F4_8462894DFBAA
#define D49F29DA_4806_421C_89F4_8462894DFBAA

#include "math/linalg/matrix_view.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Rows, sint32 Cols>
inline MatrixView<FloatType, Rows, Cols>::MatrixView(const Matrix<FloatType, Rows, Cols>& matrix,
                                                     const sint32                         rowBegin,
                                                     const sint32                         colBegin,
                                                     const sint32                         rowEnd,
                                                     const sint32                         colEnd)
    : _matrix{matrix}
    , _rowBegin{rowBegin}
    , _colBegin{colBegin}
    , _rowCount{rowEnd - rowBegin + 1}
    , _colCount{colEnd - colBegin + 1}

{
  assert(_rowBegin >= 0);
  assert(_colBegin >= 0);
  assert(_rowCount <= Rows);
  assert(_colCount <= Cols);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols>::operator()(sint32 row, sint32 col) const -> FloatType
{
  assert(row < _rowCount);
  assert(col < _colCount);
  return _matrix.at_unsafe(_rowBegin + row, _colBegin + col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols>::operator+(const Matrix<FloatType, Rows, Cols>& other) const
    -> Matrix<FloatType, Rows, Cols>
{
  auto result{other};
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result.at_unsafe(row, col) += this->operator()(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols>::operator-(const Matrix<FloatType, Rows, Cols>& other) const
    -> Matrix<FloatType, Rows, Cols>
{
  auto result{other};
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result.at_unsafe(row, col) -= this->operator()(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Cols2>
inline auto MatrixView<FloatType, Rows, Cols>::operator*(const Matrix<FloatType, Cols, Cols2>& other) const
    -> Matrix<FloatType, Rows, Cols2>
{
  Matrix<FloatType, Rows, Cols> result;
#pragma omp parallel for private(i, j, k) shared(A, B, C)
  for (auto i = 0; i < Rows; ++i)
  {
    for (auto k = 0; k < Cols; ++k)
    {
      for (auto j = 0; j < Cols2; ++j)
      {
        result.at_unsafe(i, j) += this->operator()(i, k) * other.at_unsafe(k, j);
      }
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols>::operator+(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> result;
  for (auto row = 0; row < _rowCount; ++row)
  {
    for (auto col = 0; col < _colCount; ++col)
    {
      result.at_unsafe(row, col) = this->operator()(row, col) + other;
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols>::operator-(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> result;
  for (auto row = 0; row < _rowCount; ++row)
  {
    for (auto col = 0; col < _colCount; ++col)
    {
      result.at_unsafe(row, col) = this->operator()(row, col) - other;
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols>::operator*(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> result;
  for (auto row = 0; row < _rowCount; ++row)
  {
    for (auto col = 0; col < _colCount; ++col)
    {
      result.at_unsafe(row, col) = this->operator()(row, col) * other;
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols>::operator/(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> result;
  for (auto row = 0; row < _rowCount; ++row)
  {
    for (auto col = 0; col < _colCount; ++col)
    {
      result.at_unsafe(row, col) = this->operator()(row, col) / other;
    }
  }
  return result;
}

template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
auto operator+(const MatrixView<FloatType, RowsA, ColsA>& a,
               const MatrixView<FloatType, RowsB, ColsB>& b) -> Matrix<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB)>
{
  assert(a.getRowCount() == b.getRowCount());
  assert(a.getColCount() == b.getColCount());
  Matrix<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB)> result{};

  for (auto row = 0; row < a.getRowCount(); ++row)
  {
    for (auto col = 0; col < a.getColCount(); ++col)
    {
      result.at_unsafe(row, col) = a(row, col) + b(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
auto operator-(const MatrixView<FloatType, RowsA, ColsA>& a,
               const MatrixView<FloatType, RowsB, ColsB>& b) -> Matrix<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB)>
{
  assert(a.getRowCount() == b.getRowCount());
  assert(a.getColCount() == b.getColCount());
  Matrix<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB)> result{};

  for (auto row = 0; row < a.getRowCount(); ++row)
  {
    for (auto col = 0; col < a.getColCount(); ++col)
    {
      result.at_unsafe(row, col) = a(row, col) - b(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 RowsA, sint32 ColsA, sint32 RowsB, sint32 ColsB>
auto operator*(const MatrixView<FloatType, RowsA, ColsA>& a,
               const MatrixView<FloatType, RowsB, ColsB>& b) -> Matrix<FloatType, RowsA, ColsB>
{
  assert(a.getColCount() == b.getRowCount());
  Matrix<FloatType, RowsA, ColsB> result{};

#pragma omp parallel for private(i, j, k) shared(A, B, C)
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
