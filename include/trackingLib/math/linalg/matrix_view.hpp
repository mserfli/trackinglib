#ifndef D49F29DA_4806_421C_89F4_8462894DFBAA
#define D49F29DA_4806_421C_89F4_8462894DFBAA

#include "math/linalg/matrix_view.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Rows, sint32 Cols>
inline MatrixView<FloatType, Rows, Cols, true>::MatrixView(const Matrix<FloatType, Rows, Cols>& matrix,
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
inline auto MatrixView<FloatType, Rows, Cols, true>::operator()(sint32 row, sint32 col) const -> FloatType
{
  assert(row < _rowCount);
  assert(col < _colBegin);
  return _matrix(_rowBegin + row, _colBegin + col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols, true>::operator+(const Matrix<FloatType, Rows, Cols>& other) const
    -> Matrix<FloatType, Rows, Cols>
{
  auto result{other};
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result(row, col) += this->operator()(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols, true>::operator-(const Matrix<FloatType, Rows, Cols>& other) const
    -> Matrix<FloatType, Rows, Cols>
{
  auto result{other};
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result(row, col) -= this->operator()(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Cols2>
inline auto MatrixView<FloatType, Rows, Cols, true>::operator*(const Matrix<FloatType, Cols, Cols2>& other) const
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
        result(i, j) += this->operator()(i, k) * other(k, j);
      }
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols, true>::operator+(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> result;
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result(row, col) = this->operator()(row, col) + other;
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols, true>::operator-(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> result;
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result(row, col) = this->operator()(row, col) - other;
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols, true>::operator*(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> result;
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result(row, col) = this->operator()(row, col) * other;
    }
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixView<FloatType, Rows, Cols, true>::operator/(const FloatType& other) const -> Matrix<FloatType, Rows, Cols>
{
  Matrix<FloatType, Rows, Cols> result;
  for (auto row = 0; row < Rows; ++row)
  {
    for (auto col = 0; col < Cols; ++col)
    {
      result(row, col) = this->operator()(row, col) / other;
    }
  }
  return result;
}

template <typename FloatType, sint32 RowsA, sint32 ColsA, bool ExtMemA, sint32 RowsB, sint32 ColsB, bool ExtMemB>
auto operator+(const MatrixView<FloatType, RowsA, ColsA, ExtMemA>& a, const MatrixView<FloatType, RowsB, ColsB, ExtMemB>& b)
    -> MatrixView<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB), false>
{
  assert(a.getRowCount() == b.getRowCount());
  assert(a.getColCount() == b.getColCount());
  MatrixView<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB), false> result{0, 0, a.getRowCount(), a.getColCount()};

  for (auto row = 0; row < a.getRowCount(); ++row)
  {
    for (auto col = 0; col < a.getColCount(); ++col)
    {
      result(row, col) = a(row, col) + b(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 RowsA, sint32 ColsA, bool ExtMemA, sint32 RowsB, sint32 ColsB, bool ExtMemB>
auto operator-(const MatrixView<FloatType, RowsA, ColsA, ExtMemA>& a, const MatrixView<FloatType, RowsB, ColsB, ExtMemB>& b)
    -> MatrixView<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB), false>
{
  assert(a.getRowCount() == b.getRowCount());
  assert(a.getColCount() == b.getColCount());
  MatrixView<FloatType, std::min(RowsA, RowsB), std::min(ColsA, ColsB), false> result{0, 0, a.getRowCount(), a.getColCount()};

  for (auto row = 0; row < a.getRowCount(); ++row)
  {
    for (auto col = 0; col < a.getColCount(); ++col)
    {
      result(row, col) = a(row, col) - b(row, col);
    }
  }
  return result;
}

template <typename FloatType, sint32 RowsA, sint32 ColsA, bool ExtMemA, sint32 RowsB, sint32 ColsB, bool ExtMemB>
auto operator*(const MatrixView<FloatType, RowsA, ColsA, ExtMemA>& a, const MatrixView<FloatType, RowsB, ColsB, ExtMemB>& b)
    -> MatrixView<FloatType, RowsA, ColsB, false>
{
  assert(a.getColCount() == b.getRowCount());
  MatrixView<FloatType, RowsA, ColsB, false> result{0, 0, a.getRowCount(), b.getColCount()};

#pragma omp parallel for private(i, j, k) shared(A, B, C)
  for (auto i = 0; i < result.getRowCount(); ++i)
  {
    for (auto k = 0; k < a.getColCount(); ++k)
    {
      for (auto j = 0; j < result.getColCount(); ++j)
      {
        result(i, j) += a(i, k) * b(k, j);
      }
    }
  }
}

} // namespace math
} // namespace tracking

#endif // D49F29DA_4806_421C_89F4_8462894DFBAA
