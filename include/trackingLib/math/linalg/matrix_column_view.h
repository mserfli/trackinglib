#ifndef ACBE61FE_A7AF_40EC_A62C_B94049A80274
#define ACBE61FE_A7AF_40EC_A62C_B94049A80274

#include "base/first_include.h"
#include "base/atomic_types.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace math
{
template <typename FloatType, sint32 Rows, sint32 Cols>
class MatrixColumnView
{
public:
  explicit MatrixColumnView(const Matrix<FloatType, Rows, Cols>& matrix, const sint32 col);
  auto operator[](const sint32 idx) const -> FloatType { return _matrix(idx, _col); }
  auto operator*(const Vector<FloatType, Rows>& other) const -> FloatType;

private:
  const Matrix<FloatType, Rows, Cols>& _matrix;
  const sint32                         _col;
};

template <typename FloatType, sint32 Rows, sint32 Cols>
MatrixColumnView<FloatType, Rows, Cols>::MatrixColumnView(const Matrix<FloatType, Rows, Cols>& matrix, const sint32 col)
    : _matrix{matrix}
    , _col{col}
{
}

template <typename FloatType, sint32 Rows, sint32 Cols>
auto MatrixColumnView<FloatType, Rows, Cols>::operator*(const Vector<FloatType, Rows>& other) const -> FloatType
{
  // calc dot product
  FloatType result{};
  for(auto row=0;row<Rows; ++row)
  {
    result += _matrix(row,_col) * other[row];
  }
  return result;
}

template <typename FloatType, sint32 Rows, sint32 Cols, sint32 Cols2>
auto operator*(const Matrix<FloatType, Rows, Cols>& mat, const MatrixColumnView<FloatType, Cols, Cols2>& colView)
    -> Vector<FloatType, Rows>
{
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
