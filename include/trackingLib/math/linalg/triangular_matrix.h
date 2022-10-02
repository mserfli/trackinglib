#ifndef BC7FD90F_FBB7_481C_89C4_89BEE41309C5
#define BC7FD90F_FBB7_481C_89C4_89BEE41309C5

#include "base/first_include.h"
#include "math/linalg/square_matrix.h"

namespace tracking
{
namespace math
{

// TODO(matthias): add interface contract
// TODO(matthias): speedup transpose by storing the current transpose status and swap col/row access
// TODO(matthias): use own memory optimized to required number of elements
template <typename FloatType, sint32 Size, bool isLower>
class TriangularMatrix: public SquareMatrix<FloatType, Size>
{
public:
  // rule of 5 declarations
  TriangularMatrix() = default;
  TriangularMatrix(const TriangularMatrix& other) = default;
  TriangularMatrix(TriangularMatrix&&) noexcept = default;
  auto operator=(const TriangularMatrix&) -> TriangularMatrix& = default;
  auto operator=(TriangularMatrix&&) noexcept -> TriangularMatrix& = default;

  /// \brief Construct a new Triangular Matrix object
  /// \param[in] other
  TriangularMatrix(const SquareMatrix<FloatType, Size>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new TriangularMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  TriangularMatrix(const std::initializer_list<std::initializer_list<FloatType>>& list);

  /// \brief Set a lower/upper triangular block matrix at given position
  /// \tparam SrcSize    Size of the source block
  /// \tparam SrcCount   Number of diagonal elements to copy from source
  /// \tparam SrcRowBeg  Begin row index in source
  /// \tparam SrcColBeg  Begin col index in source
  /// \tparam DstRowBeg  Begin row index in dest
  /// \tparam DstColBeg  Begin col index in dest
  /// \param[in] block   Source block matrix to copy from
  template <sint32 SrcSize, sint32 SrcCount, sint32 SrcRowBeg, sint32 SrcColBeg, sint32 DstRowBeg, sint32 DstColBeg>
  void setBlock(const TriangularMatrix<FloatType, SrcSize, isLower>& block);

  /// \brief Element read-only access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType  scalar triangular value
  auto operator()(sint32 row, sint32 col) const -> FloatType;

  /// \brief Element access to a scalar triangular value
  /// \param[in] row  Row index of the element
  /// \param[in] col  Col index of the element
  /// \return FloatType&  Reference to the scalar triangular value
  auto operator()(sint32 row, sint32 col) -> FloatType&;

  /// \brief Calculate the transposed matrix
  /// \return TriangularMatrix<FloatType, Size, !isLower>
  auto transpose() const -> TriangularMatrix<FloatType, Size, !isLower>;

  /// \brief Solver for A*x=b based on Cholesky decomposition of A
  /// \tparam Cols  Number of columns in the rhs variable b
  /// \param[in] b  Matrix describing the rhs of the equation A*x=b
  /// \return Matrix<FloatType, Size, Cols> describing x
  template <sint32 Cols>
  auto solve(const Matrix<FloatType, Size, Cols>& b) const -> Matrix<FloatType, Size, Cols>;

  /// \brief Calculates the inverse of the underlying matrix
  /// \return TriangularMatrix
  auto inverse() const -> TriangularMatrix;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround to keep following idententation
  // clang-format on
  /// \brief hide inherited transpose function
  using SquareMatrix<FloatType, Size>::transpose;

  /// \brief hide inherited operator() to prevent accessing off-triangular elements
  using SquareMatrix<FloatType, Size>::operator();
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
  // zero off-triangular elements
  for (sint32 row = 0; row < Size; ++row)
  {
    for (sint32 col = row + 1; col < Size; ++col)
    {
      const sint32 rowIdx = !isLower ? col : row;
      const sint32 colIdx = !isLower ? row : col;

      SquareMatrix<FloatType, Size>::operator()(rowIdx, colIdx) = static_cast<FloatType>(0.0);
    }
  }
}

template <typename FloatType, sint32 Size, bool isLower>
template <sint32 SrcSize, sint32 SrcCount, sint32 SrcRowBeg, sint32 SrcColBeg, sint32 DstRowBeg, sint32 DstColBeg>
void TriangularMatrix<FloatType, Size, isLower>::setBlock(const TriangularMatrix<FloatType, SrcSize, isLower>& block)
{
  static_assert(SrcCount > 1, "use scalar access operator for block copy size == 1");
  static_assert(SrcRowBeg + SrcCount <= SrcSize, "copy to many rows from src");
  static_assert(SrcColBeg + SrcCount <= SrcSize, "copy to many cols from src");

  static_assert(DstRowBeg + SrcCount <= Size, "copy to many rows to dst");
  static_assert(DstColBeg + SrcCount <= Size, "copy to many cols to dst");

  constexpr bool checkSrcAccess = isLower ? SrcRowBeg >= SrcColBeg : SrcRowBeg <= SrcColBeg;
  static_assert(checkSrcAccess, "accessing off diagonal part of src triangular matrix");
  constexpr bool checkDstAccess = isLower ? DstRowBeg >= DstColBeg : DstRowBeg <= DstColBeg;
  static_assert(checkDstAccess, "accessing off diagonal part of dst triangular matrix");

  for (sint32 row = 0; row < SrcCount; ++row)
  {
    for (sint32 col = 0; col <= row; ++col)
    {
      if (isLower) // will be optimized out because isLower can be deduced at compile time
      {
        this->operator()(DstRowBeg + row, DstColBeg + col) = block(SrcRowBeg + row, SrcColBeg + col);
      }
      else
      {
        this->operator()(DstRowBeg + col, DstColBeg + row) = block(SrcRowBeg + col, SrcColBeg + row);
      }
    }
  }
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::operator()(sint32 row, sint32 col) const -> FloatType
{
  assert((isLower ? row >= col : row <= col) && "accessing off-triangular elements");
  return SquareMatrix<FloatType, Size>::operator()(row, col);
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::operator()(sint32 row, sint32 col) -> FloatType&
{
  assert((isLower ? row >= col : row <= col) && "accessing off-triangular elements");
  return SquareMatrix<FloatType, Size>::operator()(row, col);
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::transpose() const -> TriangularMatrix<FloatType, Size, !isLower>
{
  // TODO(matthias): speedup transpose by storing the current transpose status and swap col/row access
  return TriangularMatrix<FloatType, Size, !isLower>(SquareMatrix<FloatType, Size>::transpose());
}

template <typename FloatType, sint32 Size, bool isLower>
template <sint32 Cols>
inline auto TriangularMatrix<FloatType, Size, isLower>::solve(const Matrix<FloatType, Size, Cols>& b) const
    -> Matrix<FloatType, Size, Cols>
{
  Matrix<FloatType, Size, Cols> x;
  constexpr auto                UpLoType = isLower ? Eigen::Lower : Eigen::Upper;
  // depending on UpLoType .solve does a forward/backward substitution
  x._data = this->_data.template triangularView<UpLoType>().solve(b._data);
  return x;
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::inverse() const -> TriangularMatrix
{
  return TriangularMatrix(this->solve(SquareMatrix<FloatType, Size>::Identity()));
}

} // namespace math
} // namespace tracking
#endif // BC7FD90F_FBB7_481C_89C4_89BEE41309C5
