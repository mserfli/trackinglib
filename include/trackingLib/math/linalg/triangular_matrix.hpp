#ifndef E4D3E13A_DB2E_427E_BA99_4F251275B082
#define E4D3E13A_DB2E_427E_BA99_4F251275B082

#include "math/linalg/triangular_matrix.h"

#include "math/linalg/diagonal_matrix.hpp" // IWYU pragma: keep
#include "math/linalg/matrix.hpp"          // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"   // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::TriangularMatrix(const SquareMatrix& other)
    : SquareMatrix{}
{
  // copy triangular elements from other
  for (sint32 row = 0; row < Size_; ++row)
  {
    this->at_unsafe(row, row) = other.at_unsafe(row, row);
    for (sint32 col = row + 1; col < Size_; ++col)
    {
      const sint32 rowIdx = IsLower_ ? col : row;
      const sint32 colIdx = IsLower_ ? row : col;

      this->at_unsafe(rowIdx, colIdx) = other.at_unsafe(rowIdx, colIdx);
    }
  }
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::FromList(
    const std::initializer_list<std::initializer_list<ValueType_>>& list) -> TriangularMatrix
{
  return TriangularMatrix{SquareMatrix::FromList(list)};
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
template <sint32 SrcSize, sint32 SrcCount, sint32 SrcRowBeg, sint32 SrcColBeg, sint32 DstRowBeg, sint32 DstColBeg>
inline void TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::setBlock(
    const TriangularMatrix<ValueType_, SrcSize, IsLower_, IsRowMajor_>& block)
{
  static_assert(SrcCount > 1, "use scalar access operator for block copy size == 1");
  static_assert(SrcRowBeg + SrcCount <= SrcSize, "copy to many rows from src");
  static_assert(SrcColBeg + SrcCount <= SrcSize, "copy to many cols from src");

  static_assert(DstRowBeg + SrcCount <= Size_, "copy to many rows to dst");
  static_assert(DstColBeg + SrcCount <= Size_, "copy to many cols to dst");

  constexpr bool checkSrcAccess = IsLower_ ? SrcRowBeg >= SrcColBeg : SrcRowBeg <= SrcColBeg;
  static_assert(checkSrcAccess, "accessing off diagonal part of src triangular matrix");
  constexpr bool checkDstAccess = IsLower_ ? DstRowBeg >= DstColBeg : DstRowBeg <= DstColBeg;
  static_assert(checkDstAccess, "accessing off diagonal part of dst triangular matrix");

  for (sint32 row = 0; row < SrcCount; ++row)
  {
    for (sint32 col = 0; col <= row; ++col)
    {
      if (IsLower_) // will be optimized out because isLower can be deduced at compile time
      {
        this->at_unsafe(DstRowBeg + row, DstColBeg + col) = block.at_unsafe(SrcRowBeg + row, SrcColBeg + col);
      }
      else
      {
        this->at_unsafe(DstRowBeg + col, DstColBeg + row) = block.at_unsafe(SrcRowBeg + col, SrcColBeg + row);
      }
    }
  }
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
template <sint32 Cols_, bool IsRowMajor2_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::operator*(
    const Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>& mat) const -> Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>
{
  Matrix<ValueType_, Size_, Cols_, IsRowMajor2_> result{};
  if (IsLower_)
  {
#pragma omp parallel for private(i, j, k) shared(A, B, C)
    for (auto i = 0; i < Size_; ++i)
    {
      for (auto k = 0; k <= i; ++k)
      {
        for (auto j = 0; j < Cols_; ++j)
        {
          result.at_unsafe(i, j) += this->at_unsafe(i, k) * mat.at_unsafe(k, j);
        }
      }
    }
  }
  else
  {
#pragma omp parallel for private(i, j, k) shared(A, B, C)
    for (auto i = 0; i < Size_; ++i)
    {
      for (auto k = i; k < Size_; ++k)
      {
        for (auto j = 0; j < Cols_; ++j)
        {
          result.at_unsafe(i, j) += this->at_unsafe(i, k) * mat.at_unsafe(k, j);
        }
      }
    }
  }
  return result;
} // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::operator*(const TriangularMatrix& mat) const
    -> TriangularMatrix
{
  TriangularMatrix result{};
  if (IsLower_)
  {
#pragma omp parallel for private(i, j, k) shared(A, B, C)
    for (auto i = 0; i < Size_; ++i)
    {
      for (auto k = 0; k <= i; ++k)
      {
        for (auto j = 0; j <= k; ++j)
        {
          result.at_unsafe(i, j) += this->at_unsafe(i, k) * mat.at_unsafe(k, j);
        }
      }
    }
  }
  else
  {
#pragma omp parallel for private(i, j, k) shared(A, B, C)
    for (auto i = 0; i < Size_; ++i)
    {
      for (auto k = i; k < Size_; ++k)
      {
        for (auto j = k; j < Size_; ++j)
        {
          result.at_unsafe(i, j) += this->at_unsafe(i, k) * mat.at_unsafe(k, j);
        }
      }
    }
  }
  return result;
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::operator*(
    const TriangularMatrix<ValueType_, Size_, !IsLower_, IsRowMajor_>& mat) const -> SquareMatrix
{
  SquareMatrix other{mat};
  return SquareMatrix{this->operator*(other)};
} // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::operator*(
    const DiagonalMatrix<ValueType_, Size_>& diag) const -> TriangularMatrix
{
  // each column is multiplied by the corresponding diagonal column element
  auto result{*this};
  if (IsLower_)
  {
    for (auto col = 0; col < Size_; ++col)
    {
      for (auto row = col; row < Size_; ++row)
      {
        result.at_unsafe(row, col) *= diag.at_unsafe(col);
      }
    }
  }
  else
  {
    for (auto col = 0; col < Size_; ++col)
    {
      for (auto row = 0; row <= col; ++row)
      {
        result.at_unsafe(row, col) *= diag.at_unsafe(col);
      }
    }
  }
  return result;
} // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::operator*(const ValueType_ scalar) const
    -> TriangularMatrix
{
  auto result{*this};
  result *= scalar;
  return result;
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::operator*=(const ValueType_ scalar) -> TriangularMatrix&
{
  // TODO(matthias): can be optimized as soon as elements are stored in array instead of a SquareMatrix
  if (IsLower_)
  {
    for (sint32 row = 0; row < Size_; ++row)
    {
      for (sint32 col = 0; col <= row; ++col)
      {
        this->at_unsafe(row, col) *= scalar;
      }
    }
  }
  else
  {
    for (sint32 row = 0; row < Size_; ++row)
    {
      for (sint32 col = row; col < Size_; ++col)
      {
        this->at_unsafe(row, col) *= scalar;
      }
    }
  }
  return *this;
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::operator()(sint32 row, sint32 col) const
    -> tl::expected<ValueType_, Errors>
{
  if (!((IsLower_ && (row >= col)) || (!IsLower_ && (row <= col))))
  {
    return tl::unexpected<Errors>{Errors::invalid_access_idx};
  }
  return SquareMatrix::operator()(row, col);
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::operator()(sint32 row, sint32 col)
    -> tl::expected<std::reference_wrapper<ValueType_>, Errors>
{
  if (!((IsLower_ && (row >= col)) || (!IsLower_ && (row <= col))))
  {
    return tl::unexpected<Errors>{Errors::invalid_access_idx};
  }
  return SquareMatrix::operator()(row, col);
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::at_unsafe(sint32 row, sint32 col) const -> ValueType_
{
  assert((IsLower_ ? row >= col : row <= col) && "accessing off-triangular elements");
  return SquareMatrix::at_unsafe(row, col);
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::at_unsafe(sint32 row, sint32 col) -> ValueType_&
{
  assert((IsLower_ ? row >= col : row <= col) && "accessing off-triangular elements");
  return SquareMatrix::at_unsafe(row, col);
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::transpose() const -> const transpose_type&
{
  return reinterpret_cast<const transpose_type&>(*this);
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::transpose() -> transpose_type&
{
  return reinterpret_cast<transpose_type&>(*this);
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
template <sint32 Cols_, bool IsRowMajor2_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::solve(
    const Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>& b) const -> Matrix<ValueType_, Size_, Cols_, IsRowMajor2_>
{
  Matrix<ValueType_, Size_, Cols_, IsRowMajor2_> x{};
  if (IsLower_) // LCOV_EXCL_LINE
  {
    for (auto k = 0; k < Cols_; ++k)
    { // LCOV_EXCL_LINE
      for (auto row = 0; row < Size_; ++row)
      {
        ValueType_ sum{};
        for (auto col = 0; col < row; ++col)
        {
          sum += this->at_unsafe(row, col) * x.at_unsafe(col, k);
        }
        x.at_unsafe(row, k) = (b.at_unsafe(row, k) - sum) / this->at_unsafe(row, row);
      }
    }
  }
  else // LCOV_EXCL_LINE
  {    // LCOV_EXCL_LINE
    for (auto k = 0; k < Cols_; ++k)
    { // LCOV_EXCL_LINE
      for (auto row = Size_ - 1; row >= 0; --row)
      { // LCOV_EXCL_LINE
        ValueType_ sum{};
        for (auto col = Size_ - 1; col > row; --col)
        {
          sum += this->at_unsafe(row, col) * x.at_unsafe(col, k);
        }
        x.at_unsafe(row, k) = (b.at_unsafe(row, k) - sum) / this->at_unsafe(row, row);
      }
    }
  }
  return x;
}

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::inverse() const -> TriangularMatrix
{
  return TriangularMatrix{this->solve(SquareMatrix::Identity())};
} // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
inline auto TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>::isUnitUpperTriangular() const -> bool
{
  auto isValid = true;
  for (auto idx = 0; idx < Size_; ++idx)
  {
    isValid = isValid && (static_cast<ValueType_>(1.0) == this->at_unsafe(idx, idx));
  }
  return isValid;
}


} // namespace math
} // namespace tracking

#endif // E4D3E13A_DB2E_427E_BA99_4F251275B082
