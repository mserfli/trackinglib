#ifndef ACBE61FE_A7AF_40EC_A62C_B94049A80274
#define ACBE61FE_A7AF_40EC_A62C_B94049A80274

#include "base/first_include.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{

/// \brief A column view on a Matrix object providing arithmetic operations
/// \tparam FloatType 
/// \tparam Rows 
/// \tparam Cols 
template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixColumnView final
{
public:
  /// \brief Construct a new Matrix Column View object
  /// \param[in] matrix    the viewed matrix
  /// \param[in] col       the selected column of matrix
  /// \param[in] rowBegin  optional begin row, default=0
  /// \param[in] rowCount  optional number of rows, default=Rows
  explicit MatrixColumnView(const Matrix<FloatType, Rows, Cols>& matrix,
                            const sint32                         col,
                            const sint32                         rowBegin = 0,
                            const sint32                         rowCount = Rows);

  /// \brief Read access to specific index of the column view
  /// \param[in] idx  index in range [0, rowCount]
  /// \return FloatType
  auto operator[](const sint32 idx) const -> FloatType;

  /// \brief Dot product between the viewed column and another column vector
  /// \tparam Rows2
  /// \param[in] other
  /// \return FloatType
  template <sint32 Rows2>
  auto operator*(const Vector<FloatType, Rows2>& other) const -> FloatType;

  /// \brief Get the number of rows in the column view
  /// \return sint32
  auto getRowCount() const -> sint32 { return _rowCount; }

private:
  const Matrix<FloatType, Rows, Cols>& _matrix;
  const sint32                         _col;
  const sint32                         _rowBegin;
  const sint32                         _rowCount;
};

template <typename FloatType, sint32 Rows, sint32 Cols>
MatrixColumnView<FloatType, Rows, Cols>::MatrixColumnView(const Matrix<FloatType, Rows, Cols>& matrix,
                                                          const sint32                         col,
                                                          const sint32                         rowBegin,
                                                          const sint32                         rowCount)
    : _matrix{matrix}
    , _col{col}
    , _rowBegin{rowBegin}
    , _rowCount{rowCount}
{
  assert(rowBegin + rowCount <= Rows);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
inline auto MatrixColumnView<FloatType, Rows, Cols>::operator[](const sint32 idx) const -> FloatType
{
  assert(idx < _rowCount);
  const auto row = _rowBegin + idx;
  return _matrix(row, _col);
}

template <typename FloatType, sint32 Rows, sint32 Cols>
template <sint32 Rows2>
inline auto MatrixColumnView<FloatType, Rows, Cols>::operator*(const Vector<FloatType, Rows2>& other) const -> FloatType
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

template <typename FloatType, sint32 Rows, sint32 Cols, sint32 Rows2, sint32 Cols2>
inline auto operator*(const Matrix<FloatType, Rows, Cols>& mat, const MatrixColumnView<FloatType, Rows2, Cols2>& colView)
    -> Vector<FloatType, Rows>
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

} // namespace math
} // namespace tracking

#endif // ACBE61FE_A7AF_40EC_A62C_B94049A80274
