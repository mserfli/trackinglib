#ifndef E4D3E13A_DB2E_427E_BA99_4F251275B082
#define E4D3E13A_DB2E_427E_BA99_4F251275B082

#include "math/linalg/square_matrix.h"
#include "math/linalg/triangular_matrix.h"

#include "math/linalg/diagonal_matrix.hpp"
#include "math/linalg/matrix.h"

namespace tracking
{
namespace math
{

template <typename FloatType, sint32 Size, bool isLower>
inline TriangularMatrix<FloatType, Size, isLower>::TriangularMatrix(const SquareMatrix<FloatType, Size>& other)
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
inline TriangularMatrix<FloatType, Size, isLower>::TriangularMatrix(
    const std::initializer_list<std::initializer_list<FloatType>>& list)
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
inline void TriangularMatrix<FloatType, Size, isLower>::setBlock(const TriangularMatrix<FloatType, SrcSize, isLower>& block)
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
template <sint32 Cols>
inline auto TriangularMatrix<FloatType, Size, isLower>::operator*(const Matrix<FloatType, Size, Cols>& mat) const
    -> Matrix<FloatType, Size, Cols>
{
  Matrix<FloatType, Size, Cols> result{};
  if (isLower)
  {
#pragma omp parallel for private(i, j, k) shared(A, B, C)
    for (auto i = 0; i < Size; ++i)
    {
      for (auto k = 0; k <= i; ++k)
      {
        for (auto j = 0; j < Cols; ++j)
        {
          result(i, j) += this->operator()(i, k) * mat(k, j);
        }
      }
    }
  }
  else
  {
#pragma omp parallel for private(i, j, k) shared(A, B, C)
    for (auto i = 0; i < Size; ++i)
    {
      for (auto k = i; k < Size; ++k)
      {
        for (auto j = 0; j < Cols; ++j)
        {
          result(i, j) += this->operator()(i, k) * mat(k, j);
        }
      }
    }
  }
  return result;
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::operator*(const TriangularMatrix<FloatType, Size, isLower>& mat) const
    -> TriangularMatrix
{
  TriangularMatrix result{};
  if (isLower)
  {
#pragma omp parallel for private(i, j, k) shared(A, B, C)
    for (auto i = 0; i < Size; ++i)
    {
      for (auto k = 0; k <= i; ++k)
      {
        for (auto j = 0; j <= k; ++j)
        {
          result(i, j) += this->operator()(i, k) * mat(k, j);
        }
      }
    }
  }
  else
  {
#pragma omp parallel for private(i, j, k) shared(A, B, C)
    for (auto i = 0; i < Size; ++i)
    {
      for (auto k = i; k < Size; ++k)
      {
        for (auto j = k; j < Size; ++j)
        {
          result(i, j) += this->operator()(i, k) * mat(k, j);
        }
      }
    }
  }
  return result;
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::operator*(const TriangularMatrix<FloatType, Size, !isLower>& mat) const
    -> SquareMatrix<FloatType, Size>
{
  // implementation prevents if in inner loop: j<=k
  SquareMatrix<FloatType, Size> other{mat};
  return this->operator*(other);
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::operator*(const DiagonalMatrix<FloatType, Size>& diag) const
    -> TriangularMatrix
{
  // each column is multiplied by the corresponding diagonal column element
  auto result{*this};
  if (isLower)
  {
    for (auto col = 0; col < Size; ++col)
    {
      for (auto row = col; row < Size; ++row)
      {
        result(row, col) *= diag[col];
      }
    }
  }
  else
  {
    for (auto col = 0; col < Size; ++col)
    {
      for (auto row = 0; row <= col; ++row)
      {
        result(row, col) *= diag[col];
      }
    }
  }
  return result;
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::operator*(const FloatType scalar) const -> TriangularMatrix
{
  auto result{*this};
  result *= scalar;
  return result;
}

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::operator*=(const FloatType scalar) -> TriangularMatrix&
{
  // TODO(matthias): can be optimized as soon as elements are stored in array instead of a SquareMatrix
  if (isLower)
  {
    for (sint32 row = 0; row < Size; ++row)
    {
      for (sint32 col = 0; col <= row; ++col)
      {
        this->operator()(row, col) *= scalar;
      }
    }
  }
  else
  {
    for (sint32 row = 0; row < Size; ++row)
    {
      for (sint32 col = row; col < Size; ++col)
      {
        this->operator()(row, col) *= scalar;
      }
    }
  }
  return *this;
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

template <typename FloatType, sint32 Size, bool isLower>
inline auto TriangularMatrix<FloatType, Size, isLower>::isUnitUpperTriangular() const -> bool
{
  auto isValid = true;
  for (auto idx = 0; idx < Size; ++idx)
  {
    isValid = isValid && (static_cast<FloatType>(1.0) == this->operator()(idx, idx));
  }
  return isValid;
}


} // namespace math
} // namespace tracking

#endif // E4D3E13A_DB2E_427E_BA99_4F251275B082
