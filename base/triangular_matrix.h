#ifndef BC7FD90F_FBB7_481C_89C4_89BEE41309C5
#define BC7FD90F_FBB7_481C_89C4_89BEE41309C5

#include "base/square_matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size, bool isLower>
class TriangularMatrix: public SquareMatrix<FloatType, Size>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using SquareMatrix<FloatType, Size>::SquareMatrix;

  TriangularMatrix(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new TriangularMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  TriangularMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  /// \brief Calculate the transposed matrix
  /// \return TriangularMatrix<FloatType, Size, !isLower>
  auto transpose() const -> TriangularMatrix<FloatType, Size, !isLower>;

  /// \brief Solver for A*x=b based on Cholesky decomposition of A
  /// \tparam Cols  Number of columns in the rhs variable b
  /// \param[in] b  Matrix describing the rhs of the equation A*x=b
  /// \return Matrix<FloatType, Size, Cols> describing x
  template <sint32 Cols>
  auto solve(const Matrix<FloatType, Size, Cols>& b) -> Matrix<FloatType, Size, Cols>;

private:
  using SquareMatrix<FloatType, Size>::transpose;
};

template <typename FloatType, sint32 Size, bool isLower>
TriangularMatrix<FloatType, Size, isLower>::TriangularMatrix(const SquareMatrix<FloatType, Size>& other)
    : SquareMatrix<FloatType, Size>{}
{
  // copy triangular elements from other
  for (sint32 row = 0; row < Size; ++row)
  {
    this->operator()(row, row) = other(row, row);
    for (sint32 col = row + 1; col < Size; ++col)
    {
      const sint32 rowIdx = isLower ? col : row;
      const sint32 colIdx = isLower ? row : col;

      this->operator()(rowIdx, colIdx) = other(rowIdx, colIdx);
    }
  }
}

template <typename FloatType, sint32 Size, bool isLower>
TriangularMatrix<FloatType, Size, isLower>::TriangularMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list)
    : SquareMatrix<FloatType, Size>{list}
{
  // zero anti-triangular elements
  for (sint32 row = 0; row < Size; ++row)
  {
    for (sint32 col = row + 1; col < Size; ++col)
    {
      const sint32 rowIdx = !isLower ? col : row;
      const sint32 colIdx = !isLower ? row : col;

      this->operator()(rowIdx, colIdx) = static_cast<FloatType>(0.0);
    }
  }
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::transpose() const -> TriangularMatrix<FloatType, Size, !isLower>
{
  return SquareMatrix<FloatType, Size>::transpose();
}

template <typename FloatType, sint32 Size, bool isLower>
template <sint32 Cols>
inline auto TriangularMatrix<FloatType, Size, isLower>::solve(const Matrix<FloatType, Size, Cols>& b)
    -> Matrix<FloatType, Size, Cols>
{
  Matrix<FloatType, Size, Cols> x;
  constexpr auto                UpLoType = isLower ? Eigen::Lower : Eigen::Upper;
  // depending on UpLoType .solve does a forward/backward substitution
  x._data = this->_data.template triangularView<UpLoType>().solve(b._data);
  return x;
}

} // namespace base
} // namespace tracking
#endif // BC7FD90F_FBB7_481C_89C4_89BEE41309C5
