#ifndef A45AF6CD_F19E_48A5_92B4_D7A86498C01A
#define A45AF6CD_F19E_48A5_92B4_D7A86498C01A

#include "base/first_include.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{

/// \brief A row view on a Matrix object providing arithmetic operations
/// \tparam FloatType
/// \tparam Rows
/// \tparam Cols
template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixRowView final
{
public:
  /// \brief Construct a new Matrix Row View object
  /// \param[in] matrix    the viewed matrix
  /// \param[in] row       the selected row of matrix
  /// \param[in] colBegin  optional begin col, default=0
  /// \param[in] colCount  optional number of cols, default=Cols
  explicit MatrixRowView(const Matrix<FloatType, Rows, Cols>& matrix,
                         const sint32                         row,
                         const sint32                         colBegin = 0,
                         const sint32                         colCount = Cols);

  /// \brief Read access to specific index of the row view
  /// \param[in] idx  index in range [0, colCount]
  /// \return FloatType
  auto operator[](const sint32 idx) const -> FloatType;

  template <sint32 Rows2, sint32 Cols2>
  auto operator*(const Matrix<FloatType, Rows2, Cols2>& other) const -> Matrix<FloatType, 1, Cols2>;

  /// \brief Get the number of cols in the row view
  /// \return sint32
  auto getColCount() const -> sint32 { return _colCount; }

private:
  const Matrix<FloatType, Rows, Cols>& _matrix;
  const sint32                         _row;
  const sint32                         _colBegin;
  const sint32                         _colCount;
};

template <typename FloatType, sint32 Rows, sint32 Cols>
MatrixRowView<FloatType, Rows, Cols>::MatrixRowView(const Matrix<FloatType, Rows, Cols>& matrix,
                                                    const sint32                         row,
                                                    const sint32                         colBegin,
                                                    const sint32                         colCount)
    : _matrix{matrix}
    , _row{row}
    , _colBegin{colBegin}
    , _colCount{colCount}
{
  assert(colBegin + colCount <= Cols);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixRowView<FloatType, Rows, Cols>::operator[](const sint32 idx) const -> FloatType
{
  assert(idx < _colCount);
  const auto col = _colBegin + idx;
  return _matrix(_row, col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Rows2, sint32 Cols2>
inline auto MatrixRowView<FloatType, Rows, Cols>::operator*(const Matrix<FloatType, Rows2, Cols2>& other) const
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

template <typename FloatType, sint32 Rows, sint32 Rows2, sint32 Cols2>
inline auto operator*(const Vector<FloatType, Rows>& vec, const MatrixRowView<FloatType, Rows2, Cols2>& rowView) -> FloatType
{
  static_assert(Rows <= Cols2);
  assert(Rows == rowView.getColCount());
  // calc dot product
  FloatType result{};
  for (auto row = 0; row < Rows; ++row)
  {
    result += vec[row] * rowView[row];
  }
  return result;
}

} // namespace math
} // namespace tracking

#endif // A45AF6CD_F19E_48A5_92B4_D7A86498C01A
